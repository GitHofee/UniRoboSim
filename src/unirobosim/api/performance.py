"""Optional, non-destructive API-wall-time performance snapshots.

Counters describe successful API operations, not GPU execution or CPU time. Parent
and child durations may overlap and must not be added without a boundary definition.
Readers compute deltas only within the same world, generation, and epoch. A zero
count means no samples, not a measured zero-duration operation.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from .capabilities import CapabilityId
from .values import Tick

PERFORMANCE_CAPABILITY_ID = CapabilityId("world.performance@1")
_COUNTER_NAME = re.compile(r"^[a-z][a-z0-9_]{0,63}$")


def _integer(value: object, name: str, minimum: int = 0) -> None:
    if type(value) is not int or value < minimum:
        raise ValueError(f"{name} must be an exact integer >= {minimum}")


@dataclass(frozen=True, slots=True)
class PerformanceCounter:
    name: str
    count: int
    total_ns: int

    def __post_init__(self) -> None:
        if type(self.name) is not str or not _COUNTER_NAME.fullmatch(self.name):
            raise ValueError("performance counter name must be a bounded canonical identifier")
        _integer(self.count, "count")
        _integer(self.total_ns, "total_ns")
        if self.count == 0 and self.total_ns != 0:
            raise ValueError("an unsampled counter must have zero total_ns")


@dataclass(frozen=True, slots=True)
class PerformanceSnapshot:
    world_id: str
    generation: int
    epoch: int
    tick: Tick
    captured_monotonic_ns: int
    counters: tuple[PerformanceCounter, ...]
    enabled: bool = False

    def __post_init__(self) -> None:
        if type(self.world_id) is not str or not self.world_id or len(self.world_id) > 1024 or "\x00" in self.world_id:
            raise ValueError("world_id must be a bounded non-empty string")
        _integer(self.generation, "generation", 1)
        _integer(self.epoch, "epoch")
        _integer(self.captured_monotonic_ns, "captured_monotonic_ns")
        if type(self.enabled) is not bool:
            raise TypeError("enabled must be an exact bool")
        if not self.enabled and (self.counters or self.captured_monotonic_ns != 0):
            raise ValueError("inactive performance snapshots have no counters or capture clock")
        if type(self.tick) is not Tick:
            raise TypeError("tick must be an exact Tick")
        if type(self.counters) is not tuple or len(self.counters) > 64:
            raise TypeError("counters must be an exact bounded tuple")
        if any(type(counter) is not PerformanceCounter for counter in self.counters):
            raise TypeError("counters must contain exact PerformanceCounter values")
        if len({counter.name for counter in self.counters}) != len(self.counters):
            raise ValueError("performance counter names must be unique")


@runtime_checkable
class PerformanceWorld(Protocol):
    """Default-off endpoint; observers own enable/disable lifetime and sharing."""

    def set_performance_enabled(self, enabled: bool) -> None: ...

    def performance_snapshot(self) -> PerformanceSnapshot: ...


__all__ = ["PERFORMANCE_CAPABILITY_ID", "PerformanceCounter", "PerformanceSnapshot", "PerformanceWorld"]
