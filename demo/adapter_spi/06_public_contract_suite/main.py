"""Run the teaching Adapter's public-only semantic acceptance suite."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> None:
    adapter_root = Path(__file__).resolve().parents[1] / "teaching_adapter"
    completed = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", str(adapter_root / "tests")],
        check=False,
    )
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)
    print("public_contract_suite=passed")


if __name__ == "__main__":
    main()
