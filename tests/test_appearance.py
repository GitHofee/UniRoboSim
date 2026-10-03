import json
from dataclasses import replace

import pytest

from unirobosim import (
    AppearanceBinding,
    AppearanceEnvironment,
    AppearanceLight,
    AppearanceMaterial,
    AppearanceRenderer,
    AppearanceSnapshot,
    appearance_topology_sha256,
)


def snapshot():
    return AppearanceSnapshot(
        (AppearanceMaterial("m", (0.2, 0.4, 0.6, 1), 0.8, 0.2),),
        (AppearanceBinding("/box", "body/visual:0", "m"),),
        (AppearanceLight("sun", "directional", (1, 1, 1), 5, "linear_rgb_irradiance", (-1, -1, -1)),),
        AppearanceEnvironment((0.04, 0.08, 0.12), (0.1, 0.1, 0.1)),
        AppearanceRenderer("genesis", "Rasterizer"),
    )


def test_snapshot_json_roundtrip_and_invalid_identity():
    value = snapshot()
    assert AppearanceSnapshot.from_dict(json.loads(json.dumps(value.to_dict()))) == value
    with pytest.raises(ValueError):
        replace(value.bindings[0], mesh_key=["not", "a", "key"])
    with pytest.raises(ValueError):
        replace(value, bindings=value.bindings * 2)
    with pytest.raises(ValueError):
        replace(value.lights[0], orientation_xyzw=(0, 0, 0, 0))
    with pytest.raises(ValueError):
        replace(value.materials[0], base_color_linear_rgba=(2, 0, 0, 1))


def test_uv_topology_digest_detects_vertex_reorder():
    vertices = ((0, 0, 0), (1, 0, 0), (0, 1, 0))
    a = appearance_topology_sha256(vertices, (0, 1, 2), (3,))
    b = appearance_topology_sha256(vertices[::-1], (2, 1, 0), (3,))
    assert a != b
