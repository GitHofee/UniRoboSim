"""Portable debug trace viewer command-line interface."""

from __future__ import annotations

import argparse
from pathlib import Path

from .viewer import build_portable_viewer, render_trace_svg


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build portable UniRoboSim debug evidence")
    parser.add_argument("trace", type=Path, help="canonical .urs-debug.jsonl trace")
    parser.add_argument("--output", type=Path, required=True, help="output .html or .svg path")
    parser.add_argument("--sequence", type=int, help="SVG sequence (defaults to final)")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.output.suffix.lower() == ".svg":
        report = render_trace_svg(args.trace, args.output, sequence=args.sequence)
    else:
        report = build_portable_viewer(args.trace, args.output)
    print(
        f"output={report.output_path} frames={report.frame_count} "
        f"primitives={report.primitive_count} sha256={report.sha256}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
