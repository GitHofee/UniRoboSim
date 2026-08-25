from __future__ import annotations

import ast
import os
import re
import subprocess
import sys
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT / "demo"
TEACHING_SRC = DEMO / "adapter_spi" / "teaching_adapter" / "src"

EXPECTED_CASES = {
    "easyapi": (
        "01_first_simulation",
        "02_rigid_body_state",
        "03_articulation_control",
        "04_multiple_environments",
        "05_camera_observations",
        "06_capabilities_and_provider",
        "07_asset_bundle",
        "08_debug_and_scene_snapshot",
        "09_deformable_and_fluid",
        "10_switch_native_backend",
    ),
    "runtime_api": (
        "01_provider_session_world",
        "02_capability_selection",
        "03_articulation_command",
        "04_rigid_body_and_camera",
        "05_scene_control",
        "06_transactional_build",
        "07_verified_build_input",
    ),
    "adapter_spi": (
        "01_descriptor_and_probe",
        "02_session_lifecycle",
        "03_world_and_control",
        "04_capability_honesty",
        "05_entry_point_discovery",
        "06_public_contract_suite",
        "07_clean_wheel_gate",
    ),
}


def _environment() -> dict[str, str]:
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join((str(ROOT / "src"), str(TEACHING_SRC)))
    return environment


def test_demo_has_three_unumbered_learning_paths_and_exact_numbered_cases() -> None:
    assert (DEMO / "README.md").is_file()
    assert (DEMO / "README.zh-CN.md").is_file()
    category_directories = {path.name for path in DEMO.iterdir() if path.is_dir()}
    assert category_directories == set(EXPECTED_CASES)
    for category, expected in EXPECTED_CASES.items():
        root = DEMO / category
        assert (root / "README.md").is_file()
        assert (root / "README.zh-CN.md").is_file()
        actual = tuple(sorted(path.name for path in root.iterdir() if path.is_dir() and path.name[:2].isdigit()))
        assert actual == expected


def test_every_numbered_case_has_executable_source_and_paired_documentation() -> None:
    for category, cases in EXPECTED_CASES.items():
        for case in cases:
            root = DEMO / category / case
            assert (root / "main.py").is_file(), root
            assert (root / "README.md").is_file(), root
            assert (root / "README.zh-CN.md").is_file(), root


def test_all_demo_python_is_syntax_valid_and_uses_public_core_imports() -> None:
    violations: list[str] = []
    for path in sorted(DEMO.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module:
                module = node.module
                if module.startswith("unirobosim.") and module != "unirobosim.testing":
                    violations.append(f"{path.relative_to(ROOT)}:{node.lineno}: {module}")
            elif isinstance(node, ast.Import):
                for alias in node.names:
                    if alias.name.startswith("unirobosim.") and alias.name != "unirobosim.testing":
                        violations.append(f"{path.relative_to(ROOT)}:{node.lineno}: {alias.name}")
    assert violations == []


@pytest.mark.parametrize(
    "relative",
    [
        *(Path("easyapi") / case / "main.py" for case in EXPECTED_CASES["easyapi"][:9]),
        *(Path("runtime_api") / case / "main.py" for case in EXPECTED_CASES["runtime_api"]),
        *(Path("adapter_spi") / case / "main.py" for case in EXPECTED_CASES["adapter_spi"][:4]),
    ],
)
def test_dependency_free_demo_executes(relative: Path) -> None:
    completed = subprocess.run(
        [sys.executable, str(DEMO / relative)],
        cwd=ROOT,
        env=_environment(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=20,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout


def test_teaching_adapter_public_contract_suite_passes() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", str(DEMO / "adapter_spi" / "teaching_adapter" / "tests")],
        cwd=ROOT,
        env=_environment(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=30,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout
    assert "10 passed" in completed.stdout


def test_teaching_adapter_declares_installable_backend_entry_point() -> None:
    project = tomllib.loads((DEMO / "adapter_spi" / "teaching_adapter" / "pyproject.toml").read_text(encoding="utf-8"))
    assert project["project"]["dependencies"] == ["unirobosim>=0.10,<0.11"]
    assert project["project"]["entry-points"]["unirobosim.backends"] == {
        "teaching": "unirobosim_teaching:create_provider"
    }


def test_all_local_markdown_links_resolve() -> None:
    link_pattern = re.compile(r"(?<!!)\[[^]]*]\(([^)]+)\)")
    broken: list[str] = []
    for readme in sorted(DEMO.rglob("*.md")):
        for target in link_pattern.findall(readme.read_text(encoding="utf-8")):
            value = target.strip().strip("<>")
            if not value or value.startswith(("#", "http://", "https://", "mailto:")):
                continue
            path_text = value.split("#", 1)[0]
            if path_text and not (readme.parent / path_text).resolve().exists():
                broken.append(f"{readme.relative_to(ROOT)} -> {value}")
    assert broken == []


def test_source_distribution_manifest_retains_demo_library() -> None:
    manifest = (ROOT / "MANIFEST.in").read_text(encoding="utf-8")
    assert "recursive-include demo *.json *.md *.py *.toml *.urdf" in manifest
