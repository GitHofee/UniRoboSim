# ruff: noqa: E501
"""Self-contained Portable Viewer and deterministic SVG evidence renderer."""

from __future__ import annotations

import hashlib
import html
import json
from dataclasses import dataclass
from pathlib import Path
from typing import TypeAlias, cast

from unirobosim.api.errors import ValidationError

from .bus import _matches
from .model import DebugLifetimeMode, DebugPrimitive, DebugPrimitiveKind
from .trace import DebugTrace, DebugTraceEventKind, DebugTraceReader

Vector3: TypeAlias = tuple[float, float, float]
Segment: TypeAlias = tuple[Vector3, Vector3]


@dataclass(frozen=True)
class PortableViewerReport:
    output_path: Path
    frame_count: int
    primitive_count: int
    sha256: str


def _trace(value: DebugTrace | str | Path) -> DebugTrace:
    if isinstance(value, DebugTrace):
        return value
    if isinstance(value, (str, Path)):
        return DebugTraceReader().read(value)
    raise ValidationError("viewer requires a DebugTrace or path", operation="debug_viewer.build")


def _snapshots(trace: DebugTrace) -> tuple[tuple[int, tuple[DebugPrimitive, ...]], ...]:
    active: dict[tuple[str, str, str], DebugPrimitive] = {}
    snapshots: list[tuple[int, tuple[DebugPrimitive, ...]]] = [(0, ())]
    for event in trace.events:
        if event.kind is DebugTraceEventKind.PUBLISH:
            assert event.batch is not None
            for primitive in event.batch.primitives:
                active[primitive.key] = primitive
        elif event.kind is DebugTraceEventKind.CLEAR:
            for key in tuple(active):
                if _matches(key, event.layer, event.group, event.primitive_id):
                    del active[key]
        else:
            for key, primitive in tuple(active.items()):
                if primitive.lifetime.mode is not DebugLifetimeMode.MANUAL:
                    del active[key]
        snapshots.append((event.sequence, tuple(active[key] for key in sorted(active))))
    return tuple(snapshots)


def _quaternion_rotate(vector: Vector3, quaternion_xyzw: tuple[float, float, float, float]) -> Vector3:
    x, y, z = vector
    qx, qy, qz, qw = quaternion_xyzw
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _add(left: Vector3, right: Vector3) -> Vector3:
    return left[0] + right[0], left[1] + right[1], left[2] + right[2]


def _line_geometry(primitive: DebugPrimitive) -> tuple[tuple[Segment, str], ...]:
    nested = primitive.geometry_m.nested()
    default_color = _css_color(primitive.color_rgba)
    result: list[tuple[Segment, str]] = []
    if primitive.kind is DebugPrimitiveKind.LINE_LIST:
        for environment in nested:
            for segment in cast(tuple[tuple[tuple[float, ...], tuple[float, ...]], ...], environment):
                result.append(((cast(Vector3, segment[0]), cast(Vector3, segment[1])), default_color))
    elif primitive.kind is DebugPrimitiveKind.COORDINATE_AXES:
        colors = ("#ff4d4d", "#4dff70", "#4d8dff")
        for environment in nested:
            for raw_pose in cast(tuple[tuple[float, ...], ...], environment):
                origin = cast(Vector3, tuple(float(item) for item in raw_pose[:3]))
                orientation = cast(tuple[float, float, float, float], tuple(float(item) for item in raw_pose[3:7]))
                for axis, color in zip(
                    ((primitive.size, 0.0, 0.0), (0.0, primitive.size, 0.0), (0.0, 0.0, primitive.size)),
                    colors,
                    strict=True,
                ):
                    result.append(((origin, _add(origin, _quaternion_rotate(axis, orientation))), color))
    elif primitive.kind is DebugPrimitiveKind.BOUNDING_BOX:
        edges = (
            (0, 1),
            (0, 2),
            (0, 4),
            (1, 3),
            (1, 5),
            (2, 3),
            (2, 6),
            (3, 7),
            (4, 5),
            (4, 6),
            (5, 7),
            (6, 7),
        )
        for environment in nested:
            for raw_box in cast(tuple[tuple[float, ...], ...], environment):
                center = cast(Vector3, tuple(float(item) for item in raw_box[:3]))
                half = tuple(float(item) * 0.5 for item in raw_box[3:6])
                orientation = cast(tuple[float, float, float, float], tuple(float(item) for item in raw_box[6:10]))
                corners = tuple(
                    _add(
                        center,
                        _quaternion_rotate(
                            (
                                half[0] if index & 1 else -half[0],
                                half[1] if index & 2 else -half[1],
                                half[2] if index & 4 else -half[2],
                            ),
                            orientation,
                        ),
                    )
                    for index in range(8)
                )
                result.extend(((corners[left], corners[right]), default_color) for left, right in edges)
    elif primitive.kind is DebugPrimitiveKind.TRAJECTORY:
        for environment in nested:
            points = tuple(cast(Vector3, point) for point in cast(tuple[tuple[float, ...], ...], environment))
            result.extend(((left, right), default_color) for left, right in zip(points, points[1:], strict=False))
    return tuple(result)


def _point_geometry(primitive: DebugPrimitive) -> tuple[Vector3, ...]:
    if primitive.kind is not DebugPrimitiveKind.POINT_SET:
        return ()
    return tuple(
        cast(Vector3, point)
        for environment in primitive.geometry_m.nested()
        for point in cast(tuple[tuple[float, ...], ...], environment)
    )


def _text_geometry(primitive: DebugPrimitive) -> tuple[tuple[Vector3, str], ...]:
    if primitive.kind is not DebugPrimitiveKind.TEXT:
        return ()
    assert primitive.text is not None
    nested = primitive.geometry_m.nested()
    return tuple(
        (cast(Vector3, point), primitive.text[environment_index][text_index])
        for environment_index, environment in enumerate(nested)
        for text_index, point in enumerate(cast(tuple[tuple[float, ...], ...], environment))
    )


def _css_color(color: tuple[float, float, float, float]) -> str:
    return "#{:02x}{:02x}{:02x}".format(*(round(component * 255.0) for component in color[:3]))


def _project(point: Vector3) -> tuple[float, float]:
    return 0.8660254 * (point[0] - point[1]), -point[2] + 0.5 * (point[0] + point[1])


def render_trace_svg(
    trace: DebugTrace | str | Path,
    output_path: str | Path,
    *,
    sequence: int | None = None,
    environment_indices: tuple[int, ...] | None = None,
    layers: tuple[str, ...] | None = None,
    groups: tuple[str, ...] | None = None,
    width: int = 1280,
    height: int = 720,
) -> PortableViewerReport:
    """Render one deterministic isometric SVG frame without a browser dependency."""

    if any(not isinstance(value, int) or isinstance(value, bool) or value < 64 for value in (width, height)):
        raise ValidationError("SVG dimensions must be integers of at least 64", operation="debug_viewer.svg")
    debug_trace = _trace(trace)
    snapshots = _snapshots(debug_trace)
    if sequence is None:
        snapshot_sequence, primitives = snapshots[-1]
    else:
        matches = tuple(item for item in snapshots if item[0] <= sequence)
        if not matches:
            raise ValidationError("SVG sequence is out of range", operation="debug_viewer.svg")
        snapshot_sequence, primitives = matches[-1]
    env_set = None if environment_indices is None else frozenset(environment_indices)
    selected: list[DebugPrimitive] = []
    for primitive in primitives:
        if layers is not None and primitive.layer not in layers:
            continue
        if groups is not None and primitive.group not in groups:
            continue
        selected_primitive = primitive if env_set is None else primitive.select_environments(env_set)
        if selected_primitive is not None:
            selected.append(selected_primitive)
    points: list[tuple[Vector3, str, float]] = []
    lines: list[tuple[Segment, str, float]] = []
    texts: list[tuple[Vector3, str, str, float]] = []
    for primitive in selected:
        color = _css_color(primitive.color_rgba)
        points.extend((point, color, primitive.size) for point in _point_geometry(primitive))
        lines.extend((segment, segment_color, primitive.size) for segment, segment_color in _line_geometry(primitive))
        texts.extend((point, value, color, primitive.size) for point, value in _text_geometry(primitive))
    world_points = [point for point, _, _ in points]
    world_points.extend(point for segment, _, _ in lines for point in segment)
    world_points.extend(point for point, _, _, _ in texts)
    projected = [_project(point) for point in world_points] or [(0.0, 0.0), (1.0, 1.0)]
    min_x = min(value[0] for value in projected)
    max_x = max(value[0] for value in projected)
    min_y = min(value[1] for value in projected)
    max_y = max(value[1] for value in projected)
    extent_x = max(max_x - min_x, 1.0e-6)
    extent_y = max(max_y - min_y, 1.0e-6)
    scale = min((width - 80) / extent_x, (height - 100) / extent_y)

    def screen(point: Vector3) -> tuple[float, float]:
        projected_x, projected_y = _project(point)
        return 40.0 + (projected_x - min_x) * scale, 60.0 + (projected_y - min_y) * scale

    body: list[str] = [
        f'<rect width="{width}" height="{height}" fill="#10151d"/>',
        f'<text x="24" y="32" fill="#d7e2ef" font-family="monospace" font-size="16">'
        f"{html.escape(debug_trace.manifest.run_id)} · sequence {snapshot_sequence}</text>",
    ]
    for segment, color, size in lines:
        left = screen(segment[0])
        right = screen(segment[1])
        body.append(
            f'<line x1="{left[0]:.3f}" y1="{left[1]:.3f}" x2="{right[0]:.3f}" '
            f'y2="{right[1]:.3f}" stroke="{color}" stroke-width="{max(1.0, size):.3f}"/>'
        )
    for point, color, size in points:
        x, y = screen(point)
        body.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{max(2.0, size):.3f}" fill="{color}"/>')
    for point, text_value, color, size in texts:
        x, y = screen(point)
        body.append(
            f'<text x="{x + 5.0:.3f}" y="{y - 5.0:.3f}" fill="{color}" '
            f'font-family="monospace" font-size="{max(10.0, size * 4.0):.3f}">{html.escape(text_value)}</text>'
        )
    payload = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">' + "".join(body) + "</svg>\n"
    )
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(payload, encoding="utf-8")
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    return PortableViewerReport(destination, 1, len(selected), digest)


_VIEWER_TEMPLATE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__</title><style>
:root{color-scheme:dark;font:14px system-ui,sans-serif;background:#0c1118;color:#d8e2ed}*{box-sizing:border-box}
body{margin:0;display:grid;grid-template-rows:auto 1fr;height:100vh}header{display:flex;gap:12px;align-items:center;padding:10px 14px;background:#151d28;border-bottom:1px solid #2c3948;flex-wrap:wrap}
header strong{color:#fff}label{display:flex;gap:6px;align-items:center}select{min-width:110px;max-height:74px;background:#0f1620;color:#d8e2ed;border:1px solid #344355;border-radius:4px}button,input{accent-color:#5ba8ff}button{background:#2369a8;color:white;border:0;border-radius:4px;padding:6px 12px}#viewport{min-height:0;position:relative}canvas{width:100%;height:100%;display:block;background:radial-gradient(circle at 50% 45%,#1b2734,#080c11 75%)}#status{font-family:monospace;color:#91a4b8}.hint{position:absolute;right:12px;bottom:10px;color:#71849a;font-size:12px;pointer-events:none}
</style></head><body><header><strong>UniRoboSim Portable Viewer</strong><button id="play">Play</button><label>Frame <input id="frame" type="range" min="0" value="0"></label><span id="status"></span><label>Env<select id="env" multiple></select></label><label>Layer<select id="layer" multiple></select></label><label>Group<select id="group" multiple></select></label></header><main id="viewport"><canvas id="canvas"></canvas><div class="hint">drag to rotate · wheel to zoom</div></main>
<script>"use strict";const DATA=__DATA__;const canvas=document.getElementById("canvas"),ctx=canvas.getContext("2d"),frame=document.getElementById("frame"),status=document.getElementById("status"),play=document.getElementById("play"),envSel=document.getElementById("env"),layerSel=document.getElementById("layer"),groupSel=document.getElementById("group");let yaw=-.65,pitch=.55,zoom=1,playing=false,timer=null,drag=null;
frame.max=String(DATA.frames.length-1);function options(select,values){for(const value of values){const o=document.createElement("option");o.value=value;o.textContent=value;o.selected=true;select.appendChild(o)}}options(envSel,DATA.envs.map(String));options(layerSel,DATA.layers);options(groupSel,DATA.groups);function chosen(select){return new Set([...select.selectedOptions].map(o=>o.value))}
function rot(p){const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),x=cy*p[0]-sy*p[1],y=sy*p[0]+cy*p[1];return[x,cp*y-sp*p[2],sp*y+cp*p[2]]}function qrot(v,q){const [x,y,z]=v,[qx,qy,qz,qw]=q,tx=2*(qy*z-qz*y),ty=2*(qz*x-qx*z),tz=2*(qx*y-qy*x);return[x+qw*tx+qy*tz-qz*ty,y+qw*ty+qz*tx-qx*tz,z+qw*tz+qx*ty-qy*tx]}
function add(a,b){return[a[0]+b[0],a[1]+b[1],a[2]+b[2]]}function rgba(c){return`rgba(${Math.round(c[0]*255)},${Math.round(c[1]*255)},${Math.round(c[2]*255)},${c[3]})`}function expand(p,envs){const lines=[],points=[],texts=[];for(let e=0;e<p.environment_indices.length;e++){if(!envs.has(String(p.environment_indices[e])))continue;const g=p.geometry_m[e],color=rgba(p.color_rgba);if(p.kind==="point_set")for(const v of g)points.push([v,color,p.size]);else if(p.kind==="line_list")for(const s of g)lines.push([s[0],s[1],color,p.size]);else if(p.kind==="trajectory")for(let i=1;i<g.length;i++)lines.push([g[i-1],g[i],color,p.size]);else if(p.kind==="coordinate_axes")for(const pose of g){const o=pose.slice(0,3),q=pose.slice(3,7),axes=[[[p.size,0,0],"#ff4d4d"],[[0,p.size,0],"#4dff70"],[[0,0,p.size],"#4d8dff"]];for(const [axis,c] of axes)lines.push([o,add(o,qrot(axis,q)),c,p.size])}else if(p.kind==="bounding_box")for(const b of g){const c=b.slice(0,3),h=b.slice(3,6).map(v=>v/2),q=b.slice(6,10),corners=[];for(let i=0;i<8;i++)corners.push(add(c,qrot([i&1?h[0]:-h[0],i&2?h[1]:-h[1],i&4?h[2]:-h[2]],q)));for(const [a,z] of [[0,1],[0,2],[0,4],[1,3],[1,5],[2,3],[2,6],[3,7],[4,5],[4,6],[5,7],[6,7]])lines.push([corners[a],corners[z],color,p.size])}else if(p.kind==="text")for(let i=0;i<g.length;i++)texts.push([g[i],p.text[e][i],color,p.size])}return{lines,points,texts}}
function render(){const dpr=devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;canvas.width=Math.max(1,w*dpr);canvas.height=Math.max(1,h*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);const f=DATA.frames[Number(frame.value)],envs=chosen(envSel),layers=chosen(layerSel),groups=chosen(groupSel),all={lines:[],points:[],texts:[]},world=[];for(const p of f.primitives){if(!layers.has(p.layer)||!groups.has(p.group))continue;const x=expand(p,envs);all.lines.push(...x.lines);all.points.push(...x.points);all.texts.push(...x.texts)}for(const x of all.lines)world.push(x[0],x[1]);for(const x of all.points)world.push(x[0]);for(const x of all.texts)world.push(x[0]);const rr=world.map(rot);let scale=80*zoom;if(rr.length){const xs=rr.map(p=>p[0]),ys=rr.map(p=>p[1]),dx=Math.max(...xs)-Math.min(...xs),dy=Math.max(...ys)-Math.min(...ys);scale=Math.min((w-80)/Math.max(dx,.1),(h-80)/Math.max(dy,.1))*zoom}const p=v=>{const r=rot(v);return[w/2+r[0]*scale,h/2-r[1]*scale]};ctx.lineCap="round";for(const [a,b,c,s] of all.lines){const x=p(a),y=p(b);ctx.strokeStyle=c;ctx.lineWidth=Math.max(1,s);ctx.beginPath();ctx.moveTo(...x);ctx.lineTo(...y);ctx.stroke()}for(const [v,c,s] of all.points){const x=p(v);ctx.fillStyle=c;ctx.beginPath();ctx.arc(x[0],x[1],Math.max(2,s),0,Math.PI*2);ctx.fill()}for(const [v,t,c,s] of all.texts){const x=p(v);ctx.fillStyle=c;ctx.font=`${Math.max(11,s*4)}px monospace`;ctx.fillText(t,x[0]+5,x[1]-5)}status.textContent=`${f.sequence}/${DATA.frames.at(-1).sequence} · ${f.primitives.length} active`;window.__URS_RENDER_STATS__={sequence:f.sequence,lines:all.lines.length,points:all.points.length,texts:all.texts.length}}
function stop(){playing=false;play.textContent="Play";if(timer)clearInterval(timer);timer=null}play.onclick=()=>{if(playing){stop();return}playing=true;play.textContent="Pause";timer=setInterval(()=>{let n=Number(frame.value)+1;if(n>=DATA.frames.length){stop();return}frame.value=String(n);render()},120)};for(const e of [frame,envSel,layerSel,groupSel])e.oninput=render;canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};canvas.onpointermove=e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(-1.45,Math.min(1.45,pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];render()};canvas.onpointerup=()=>drag=null;canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(.2,Math.min(5,zoom*Math.exp(-e.deltaY*.001)));render()};window.onresize=render;const query=new URLSearchParams(location.search);if(query.has("frame"))frame.value=String(Math.max(0,Math.min(DATA.frames.length-1,Number(query.get("frame")))));render();window.__URS_VIEWER_READY__=true;
</script></body></html>"""


def build_portable_viewer(
    trace: DebugTrace | str | Path,
    output_path: str | Path,
    *,
    title: str = "UniRoboSim Portable Debug Viewer",
) -> PortableViewerReport:
    """Build one offline, self-contained HTML replay artifact."""

    if not isinstance(title, str) or not title.strip() or len(title) > 200:
        raise ValidationError("viewer title must be a bounded non-empty string", operation="debug_viewer.build")
    debug_trace = _trace(trace)
    snapshots = _snapshots(debug_trace)
    frames = [
        {"sequence": sequence, "primitives": [primitive.to_dict() for primitive in primitives]}
        for sequence, primitives in snapshots
    ]
    data = {
        "schema": debug_trace.manifest.schema_version,
        "run_id": debug_trace.manifest.run_id,
        "frames": frames,
        "layers": debug_trace.manifest.layers,
        "groups": debug_trace.manifest.groups,
        "envs": debug_trace.manifest.environment_indices,
    }
    encoded = json.dumps(data, separators=(",", ":"), ensure_ascii=False).replace("</", "<\\/")
    payload = _VIEWER_TEMPLATE.replace("__TITLE__", html.escape(title)).replace("__DATA__", encoded)
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(payload, encoding="utf-8")
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    maximum = max((len(item[1]) for item in snapshots), default=0)
    return PortableViewerReport(destination, len(snapshots), maximum, digest)
