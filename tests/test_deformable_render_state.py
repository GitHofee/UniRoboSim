import struct
from dataclasses import replace

import pytest

from tests.test_soft_matter_specs import surface_body
from unirobosim import (
    ArrayValue,
    CommandError,
    DeformableTopologySnapshot,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    PackedFloat32Array,
    RenderDeformableState,
    RenderStateFrame,
    ValidationError,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def test_deformable_selected_environment_round_trip_and_atomic_rejection():
    session = FakeProvider().open()
    try:
        world = session.build(
            WorldSpec(
                "soft",
                (EntitySpec(EntityPath("/cloth"), EntityKind.SURFACE_DEFORMABLE, deformable=surface_body()),),
                environments=EnvironmentSpec(2),
            )
        )
        handle = world.resolve(EntityPath("/cloth"))
        before = world.read_deformable(handle)
        value = PackedFloat32Array((1, 4, 3), struct.pack("<12f", *range(12)))
        update = RenderDeformableState(handle, value, environment_indices=(1,))
        result = world.apply_render_state(RenderStateFrame(deformables=(update,)))
        after = world.read_deformable(handle)
        assert after.tick == before.tick == result.tick
        assert result.deformable_count == 1
        assert after.node_positions_m.nested()[0] == before.node_positions_m.nested()[0]
        assert after.node_positions_m.nested()[1][3] == (9.0, 10.0, 11.0)
        with pytest.raises(CommandError):
            world.apply_render_state(
                RenderStateFrame(
                    deformables=(
                        replace(update, positions_m=PackedFloat32Array((1, 3, 3), struct.pack("<9f", *range(9)))),
                    )
                )
            )
        assert world.read_deformable(handle) == after
        with pytest.raises(ValueError):
            RenderStateFrame(deformables=(update, update))
    finally:
        session.close()


def test_actual_topology_snapshot_validates_indices():
    spec = surface_body()
    value = DeformableTopologySnapshot(spec.topology, spec.rest_positions_m, spec.surface_triangles)
    assert value.node_count == 4
    with pytest.raises(ValidationError):
        replace(value, surface_triangles=ArrayValue.from_rows(((0, 1, 99),), dtype="int64"))
