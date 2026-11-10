"""Explicit, bounded render-only visibility changes on pinned static scenes."""

from dataclasses import dataclass
import re
from typing import Protocol, runtime_checkable
from .capabilities import CapabilityId
from .values import EntityPath

RENDER_VISIBILITY_CAPABILITY_ID = CapabilityId("render.visibility.apply@1")
_PATH = re.compile(r"[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*")


@dataclass(frozen=True, slots=True)
class RenderVisibilityOverride:
    entity_path: EntityPath
    relative_prim_path: str
    expected_visibility: str = "invisible"
    visibility: str = "inherited"
    environment_index: int = 0

    def __post_init__(self):
        if type(self.entity_path) is not EntityPath:
            raise TypeError("visibility requires an exact EntityPath")
        if (
            type(self.relative_prim_path) is not str
            or len(self.relative_prim_path) > 1024
            or not _PATH.fullmatch(self.relative_prim_path)
        ):
            raise ValueError("visibility requires a bounded relative prim path without traversal")
        if self.expected_visibility != "invisible" or self.visibility != "inherited":
            raise ValueError("only explicit invisible to inherited visibility is supported")
        if type(self.environment_index) is not int or self.environment_index < 0:
            raise ValueError("visibility environment index must be nonnegative")


@dataclass(frozen=True, slots=True)
class RenderVisibilityReceipt:
    override: RenderVisibilityOverride
    native_prim_path: str
    before_visibility: str
    before_effective_visibility: str
    visibility: str
    effective_visibility: str

    def __post_init__(self):
        if type(self.override) is not RenderVisibilityOverride:
            raise TypeError("visibility receipt requires exact override")
        if type(self.native_prim_path) is not str or not self.native_prim_path.startswith("/"):
            raise ValueError("visibility receipt requires absolute native path")
        if (self.before_visibility, self.before_effective_visibility, self.visibility, self.effective_visibility) != (
            "invisible",
            "invisible",
            "inherited",
            "inherited",
        ):
            raise ValueError("visibility receipt does not prove requested change")


@runtime_checkable
class RenderVisibilityWorld(Protocol):
    def apply_render_visibility(
        self, overrides: tuple[RenderVisibilityOverride, ...]
    ) -> tuple[RenderVisibilityReceipt, ...]: ...
