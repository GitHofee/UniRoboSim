from dataclasses import FrozenInstanceError

import pytest

from unirobosim import PERFORMANCE_CAPABILITY_ID, PerformanceCounter, PerformanceSnapshot, PerformanceWorld, Tick


def test_optional_immutable_performance_contract() -> None:
    assert PERFORMANCE_CAPABILITY_ID.value == "world.performance@1"
    counter = PerformanceCounter("physics", 2, 123)
    snapshot = PerformanceSnapshot("world", 1, 0, Tick(2, 0.1), 1000, (counter,), enabled=True)
    assert snapshot.counters == (counter,)
    with pytest.raises(FrozenInstanceError):
        snapshot.epoch = 1  # type: ignore[misc]

    class OldWorld:
        pass

    class MeasuredWorld:
        def set_performance_enabled(self, enabled: bool) -> None:
            pass

        def performance_snapshot(self) -> PerformanceSnapshot:
            return snapshot

    assert not isinstance(OldWorld(), PerformanceWorld)
    assert isinstance(MeasuredWorld(), PerformanceWorld)


@pytest.mark.parametrize(
    "values",
    [("", 0, 0), ("UPPER", 0, 0), ("physics", True, 0), ("physics", -1, 0), ("physics", 1, -1), ("physics", 0, 1)],
)
def test_counter_rejects_invalid_values(values: tuple[object, ...]) -> None:
    with pytest.raises((TypeError, ValueError)):
        PerformanceCounter(*values)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "field,value",
    [
        ("world_id", ""),
        ("generation", 0),
        ("epoch", True),
        ("enabled", 1),
        ("captured_monotonic_ns", -1),
        ("tick", (0, 0.0)),
        ("counters", []),
        ("counters", (object(),)),
        ("counters", (PerformanceCounter("frame", 0, 0),) * 2),
    ],
)
def test_snapshot_rejects_invalid_values(field: str, value: object) -> None:
    values = dict(
        world_id="world", generation=1, epoch=0, tick=Tick(0, 0.0), captured_monotonic_ns=0, counters=(), enabled=True
    )
    values[field] = value
    with pytest.raises((TypeError, ValueError)):
        PerformanceSnapshot(**values)  # type: ignore[arg-type]


def test_inactive_snapshot_is_explicit_not_a_zero_measurement() -> None:
    inactive = PerformanceSnapshot("world", 1, 0, Tick(0, 0.0), 0, ())
    assert inactive.enabled is False
    with pytest.raises(ValueError, match="inactive"):
        PerformanceSnapshot("world", 1, 0, Tick(0, 0.0), 10, ())
    with pytest.raises(ValueError, match="inactive"):
        PerformanceSnapshot("world", 1, 0, Tick(0, 0.0), 0, (PerformanceCounter("frame", 0, 0),))
