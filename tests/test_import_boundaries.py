from __future__ import annotations

import ast
import importlib
import importlib.metadata
import tomllib
import unittest
from pathlib import Path

import unirobosim

REPOSITORY = Path(__file__).resolve().parents[1]
SOURCE = REPOSITORY / "src" / "unirobosim"
FORBIDDEN_ROOTS = {"fastsim", "isaaclab", "isaacsim", "omni", "mujoco", "pybullet", "torch", "numpy"}
RELEASE_VERSION = "0.10.5"
PLANNING_RELEASE_SYMBOLS = (
    "PlanningGeometryLease",
    "PlanningGeometryResourceDescriptor",
    "PlanningGeometryResourceLayout",
    "PlanningSceneCatalog",
    "PlanningSceneDelta",
    "PlanningSceneState",
    "PlanningSceneWorld",
)


class ImportBoundaryTests(unittest.TestCase):
    def test_core_has_no_framework_or_backend_imports(self) -> None:
        violations = []
        for path in sorted(SOURCE.rglob("*.py")):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                names = []
                if isinstance(node, ast.Import):
                    names = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom) and node.module:
                    names = [node.module]
                for name in names:
                    if name.split(".", 1)[0] in FORBIDDEN_ROOTS:
                        violations.append(f"{path.relative_to(REPOSITORY)}:{node.lineno}: {name}")
        self.assertEqual(violations, [])

    def test_api_and_runtime_do_not_import_testing_package(self) -> None:
        violations = []
        for package in (SOURCE / "api", SOURCE / "runtime"):
            for path in package.rglob("*.py"):
                if "unirobosim.testing" in path.read_text(encoding="utf-8"):
                    violations.append(str(path.relative_to(REPOSITORY)))
        self.assertEqual(violations, [])

    def test_public_import_does_not_expose_fake_backend(self) -> None:
        self.assertEqual(unirobosim.__version__, RELEASE_VERSION)
        self.assertFalse(hasattr(unirobosim, "FakeProvider"))
        module = importlib.import_module("unirobosim.testing")
        self.assertTrue(hasattr(module, "FakeProvider"))

    def test_release_identity_is_exact_and_consistent(self) -> None:
        project = tomllib.loads((REPOSITORY / "pyproject.toml").read_text(encoding="utf-8"))
        testing = importlib.import_module("unirobosim.testing")
        self.assertEqual(project["project"]["version"], RELEASE_VERSION)
        self.assertEqual(importlib.metadata.version("unirobosim"), RELEASE_VERSION)
        self.assertEqual(unirobosim.__version__, RELEASE_VERSION)
        self.assertEqual(testing.FAKE_DESCRIPTOR.version, RELEASE_VERSION)

    def test_planning_release_symbols_are_public_at_both_api_levels(self) -> None:
        api = importlib.import_module("unirobosim.api")
        planning_scene = importlib.import_module("unirobosim.api.planning_scene")
        for name in PLANNING_RELEASE_SYMBOLS:
            self.assertIn(name, api.__all__)
            self.assertIn(name, unirobosim.__all__)
            self.assertIs(getattr(api, name), getattr(planning_scene, name))
            self.assertIs(getattr(unirobosim, name), getattr(planning_scene, name))

    def test_repository_has_no_second_documentation_tree(self) -> None:
        self.assertFalse((REPOSITORY / "docs").exists())

    def test_source_contains_no_automated_authorship_markers(self) -> None:
        trailer = "co-" + "authored-by: "
        generated = "machine-" + "generated authorship"
        markers = (trailer, generated)
        matches = []
        for path in REPOSITORY.rglob("*"):
            if not path.is_file() or ".git" in path.parts or "__pycache__" in path.parts:
                continue
            try:
                content = path.read_text(encoding="utf-8").lower()
            except UnicodeDecodeError:
                continue
            for marker in markers:
                if marker in content:
                    matches.append(f"{path.relative_to(REPOSITORY)}: {marker}")
        self.assertEqual(matches, [])


if __name__ == "__main__":
    unittest.main()
