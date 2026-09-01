from __future__ import annotations

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
RELEASE_ROWS = (
    "| `unirobosim` | `v0.10.0` | `>=3.11,<3.13` | Core |",
    "| `unirobosim-isaaclab` | `v0.10.14` | `>=3.12,<3.13` | `unirobosim>=0.10.4,<0.11` |",
    "| `unirobosim-mujoco` | `v0.9.3` | `>=3.12,<3.13` | `unirobosim>=0.9,<0.11` |",
    "| `unirobosim-pybullet` | `v0.9.3` | `>=3.11,<3.12` | `unirobosim>=0.9,<0.11` |",
    "| `unirobosim-usd-converter` | `v0.10.0` | `>=3.11,<3.13` | `unirobosim>=0.10,<0.11` |",
    "| `unirobosim-mcp` | `v0.10.0` | `>=3.11,<3.13` | `unirobosim>=0.10,<0.11` |",
)


@pytest.mark.parametrize("readme", ("README.md", "README.zh-CN.md"))
def test_public_readme_has_the_verified_release_matrix(readme: str) -> None:
    text = (ROOT / readme).read_text(encoding="utf-8")

    for row in RELEASE_ROWS:
        assert row in text


@pytest.mark.parametrize("readme", ("README.md", "README.zh-CN.md"))
def test_unpublished_studio_is_not_in_install_commands(readme: str) -> None:
    text = (ROOT / readme).read_text(encoding="utf-8")

    assert "git clone https://github.com/GitHofee/UniRoboSim-studio.git" not in text
    assert "python -m pip install ./UniRoboSim-studio" not in text
    assert "`unirobosim-studio`" in text
