import struct
from dataclasses import replace

import pytest

from unirobosim import (
    ArrayValue,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    PackedFloat32Array,
    ParticleFluidSpec,
    RenderParticleFluidState,
    RenderStateFrame,
    ValidationError,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def test_float32_color_contract_and_selected_range_atomicity():
    colors = ArrayValue.from_nested(((1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 0.5)), dtype="float32")
    fluid = ParticleFluidSpec(
        ArrayValue.from_nested(((0.0, 0.0, 1.0), (0.1, 0.0, 1.0))), initial_particle_colors_rgba=colors
    )
    for invalid in (
        replace(colors, dtype="float64"),
        ArrayValue.from_nested(((2.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0)), dtype="float32"),
    ):
        with pytest.raises(ValueError):
            replace(fluid, initial_particle_colors_rgba=invalid)
    session = FakeProvider().open()
    try:
        world = session.build(
            WorldSpec(
                "colors",
                (EntitySpec(EntityPath("/fluid"), EntityKind.PARTICLE_FLUID, particle_fluid=fluid),),
                environments=EnvironmentSpec(2),
            )
        )
        handle = world.resolve(EntityPath("/fluid"))
        before = world.read_particle_fluid(handle)
        update = RenderParticleFluidState(
            handle,
            ArrayValue.from_nested((((0.2, 0.0, 1.0),),)),
            environment_indices=(1,),
            first_particle_index=1,
            colors_rgba=PackedFloat32Array((1, 1, 4), struct.pack("<4f", 0.0, 0.0, 1.0, 0.25)),
        )
        world.apply_render_state(RenderStateFrame(particle_fluids=(update,)))
        after = world.read_particle_fluid(handle)
        assert after.particle_colors_rgba.nested()[0] == before.particle_colors_rgba.nested()[0]
        assert after.particle_colors_rgba.nested()[1][1] == (0.0, 0.0, 1.0, 0.25)
        assert after.tick == before.tick
        bad = replace(update, colors_rgba=PackedFloat32Array((1, 1, 4), struct.pack("<4f", 0.0, 0.0, 2.0, 1.0)))
        with pytest.raises(ValidationError):
            world.apply_render_state(RenderStateFrame(particle_fluids=(bad,)))
        assert world.read_particle_fluid(handle) == after
    finally:
        session.close()
