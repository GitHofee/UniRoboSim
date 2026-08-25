"""Build Core and Adapter wheels, then test discovery in a temporary clean venv."""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path


def run(*arguments: str) -> None:
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    subprocess.run(arguments, check=True, env=environment)


def main() -> None:
    repository = Path(__file__).resolve().parents[3]
    adapter = repository / "demo" / "adapter_spi" / "teaching_adapter"
    smoke = Path(__file__).with_name("discovery_smoke.py")

    with tempfile.TemporaryDirectory(prefix="unirobosim-wheel-gate-") as temporary:
        root = Path(temporary)
        wheels = root / "wheels"
        wheels.mkdir()
        run(
            sys.executable,
            "-m",
            "pip",
            "wheel",
            "--no-deps",
            "--no-build-isolation",
            "--wheel-dir",
            str(wheels),
            str(repository),
        )
        run(
            sys.executable,
            "-m",
            "pip",
            "wheel",
            "--no-deps",
            "--no-build-isolation",
            "--wheel-dir",
            str(wheels),
            str(adapter),
        )

        environment = root / "venv"
        run(sys.executable, "-m", "venv", str(environment))
        python = environment / "bin" / "python"
        built_wheels = tuple(str(path) for path in sorted(wheels.glob("*.whl")))
        assert len(built_wheels) == 2, built_wheels
        run(str(python), "-m", "pip", "install", "--no-index", *built_wheels)
        run(str(python), str(smoke))

    print("clean_wheel_gate=passed")


if __name__ == "__main__":
    main()
