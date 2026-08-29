"""Optional backend-encoded sensor media contracts.

The ordinary :class:`~unirobosim.api.protocols.World` contract continues to
return numeric sensor arrays.  Backends may additionally implement
``EncodedSensorWorld`` so recorders and remote transports can avoid an
unnecessary device-to-host RGB transfer followed by CPU encoding.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from .reports import ArticulationState
from .values import EntityHandle, Tick

_ENCODINGS = frozenset({"jpeg", "png"})
_COLOR_SPACES = frozenset({"srgb"})
_CHROMA_SUBSAMPLING = frozenset({"4:4:4"})


@dataclass(frozen=True, slots=True)
class EncodedSensorRequest:
    """One exact encoded camera product requested from a backend."""

    handle: EntityHandle
    encoding: str = "jpeg"
    quality: int | None = 92
    color_space: str = "srgb"
    chroma_subsampling: str | None = "4:4:4"

    def __post_init__(self) -> None:
        if not isinstance(self.handle, EntityHandle):
            raise TypeError("encoded sensor request handle must be EntityHandle")
        if self.encoding not in _ENCODINGS:
            raise ValueError("encoded sensor request encoding must be jpeg or png")
        if self.color_space not in _COLOR_SPACES:
            raise ValueError("encoded sensor request color_space must be srgb")
        if self.encoding == "jpeg":
            if type(self.quality) is not int or not 1 <= self.quality <= 100:
                raise ValueError("JPEG encoded sensor quality must be an exact integer in [1, 100]")
            if self.chroma_subsampling not in _CHROMA_SUBSAMPLING:
                raise ValueError("JPEG encoded sensor chroma_subsampling must be 4:4:4")
        elif self.quality is not None or self.chroma_subsampling is not None:
            raise ValueError("PNG encoded sensor requests do not accept JPEG options")


@dataclass(frozen=True, slots=True)
class EncodedSensorFrame:
    """Immutable encoded bytes captured for one camera at one native tick."""

    handle: EntityHandle
    payload: bytes
    width_px: int
    height_px: int
    encoding: str
    quality: int | None
    color_space: str
    chroma_subsampling: str | None
    tick: Tick

    def __post_init__(self) -> None:
        if not isinstance(self.handle, EntityHandle) or not isinstance(self.tick, Tick):
            raise TypeError("encoded sensor frame handle and tick are invalid")
        if type(self.payload) is not bytes or not self.payload:
            raise TypeError("encoded sensor frame payload must be non-empty immutable bytes")
        if type(self.width_px) is not int or type(self.height_px) is not int:
            raise TypeError("encoded sensor frame dimensions must be exact integers")
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("encoded sensor frame dimensions must be positive")
        EncodedSensorRequest(
            handle=self.handle,
            encoding=self.encoding,
            quality=self.quality,
            color_space=self.color_space,
            chroma_subsampling=self.chroma_subsampling,
        )
        if self.encoding == "jpeg" and not self.payload.startswith(b"\xff\xd8\xff"):
            raise ValueError("encoded JPEG sensor frame has no JPEG start marker")
        if self.encoding == "png" and not self.payload.startswith(b"\x89PNG\r\n\x1a\n"):
            raise ValueError("encoded PNG sensor frame has no PNG signature")


@runtime_checkable
class EncodedSensorWorld(Protocol):
    """Optional additive batch-encoding surface implemented by capable worlds."""

    def read_encoded_sensors(
        self,
        requests: Iterable[EncodedSensorRequest],
    ) -> tuple[EncodedSensorFrame, ...]: ...

    def step_and_read_articulations_and_encoded_sensors(
        self,
        articulation_handles: Iterable[EntityHandle],
        sensor_requests: Iterable[EncodedSensorRequest],
        count: int = 1,
    ) -> tuple[Tick, tuple[ArticulationState, ...], tuple[EncodedSensorFrame, ...]]: ...


__all__ = ("EncodedSensorFrame", "EncodedSensorRequest", "EncodedSensorWorld")
