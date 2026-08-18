from __future__ import annotations

import ast
import importlib
import unittest
from pathlib import Path

import unirobosim

REPOSITORY = Path(__file__).resolve().parents[1]
SOURCE = REPOSITORY / "src" / "unirobosim"
FORBIDDEN_ROOTS = {"fastsim", "isaaclab", "isaacsim", "omni", "mujoco", "pybullet", "torch", "numpy"}


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
        self.assertEqual(unirobosim.__version__, "0.1.0a0")
        self.assertFalse(hasattr(unirobosim, "FakeProvider"))
        module = importlib.import_module("unirobosim.testing")
        self.assertTrue(hasattr(module, "FakeProvider"))

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
