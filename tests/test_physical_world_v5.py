from __future__ import annotations

import hashlib
from dataclasses import replace

import pytest

from unirobosim import (
    ARTICULATION_AXIS_UNITS_MISMATCH,
    ARTICULATION_POSITION_AXIS_UNITS_UNSUPPORTED,
    COMPOSITE_WORLD_SCHEMA_VERSION,
    PHYSICAL_WORLD_SCHEMA_VERSION,
    WORLD_SCHEMA_UNSUPPORTED,
    WORLD_SCHEMA_VERSION,
    ArrayValue,
    ArticulationCommand,
    BoxGeometrySpec,
    CapabilityId,
    CapabilityNegotiationError,
    CapabilityRequirement,
    CapabilitySet,
    CommandError,
    CommandMode,
    EntityKind,
    EntityPath,
    EntitySpec,
    FrozenMap,
    PhysicsSpec,
    PlanningCompoundGeometry,
    PlanningGeometryRepresentation,
    PlanningPrimitiveGeometry,
    ProviderDescriptor,
    UnsupportedCapabilityError,
    ValidationError,
    WorldSpec,
)
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider, FakeSession, FakeWorld


def _mixed_world(*, scale: float = 1.0) -> WorldSpec:
    return WorldSpec(
        "mixed-world",
        (
            EntitySpec(
                EntityPath("/robot"),
                EntityKind.ARTICULATION,
                joint_names=("hinge", "slide"),
                initial_joint_positions=(0.25, 0.1),
                joint_position_units=("rad", "m"),
                scale_xyz=(scale, scale, scale),
            ),
        ),
        physics=PhysicsSpec(0.01, 1, (0.0, 0.0, -9.81)),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
    )


def _descriptor_without(capability: str, *, schemas: tuple[str, ...] | None = None) -> ProviderDescriptor:
    declarations = tuple(item for item in FAKE_DESCRIPTOR.capabilities if item.capability != CapabilityId(capability))
    return ProviderDescriptor(
        FAKE_DESCRIPTOR.provider_id,
        FAKE_DESCRIPTOR.display_name,
        FAKE_DESCRIPTOR.version,
        FAKE_DESCRIPTOR.contract_version,
        CapabilitySet(declarations),
        FAKE_DESCRIPTOR.supported_world_schema_versions if schemas is None else schemas,
        FAKE_DESCRIPTOR.metadata,
    )


def test_complete_v4_and_v5_world_goldens_are_exact() -> None:
    v4 = WorldSpec(
        "golden-v4",
        (
            EntitySpec(
                EntityPath("/hinged"),
                EntityKind.ARTICULATION,
                joint_names=("hinge",),
                initial_joint_positions=(0.25,),
            ),
        ),
        physics=PhysicsSpec(0.01, 1, (0.0, 0.0, -9.81)),
        schema_version=WORLD_SCHEMA_VERSION,
    )
    v5 = replace(
        _mixed_world(),
        world_id="golden-v5",
        entities=(replace(_mixed_world().entities[0], path=EntityPath("/mixed")),),
    )
    assert len(v4.canonical_json.encode()) == 507
    assert hashlib.sha256(v4.canonical_json.encode()).hexdigest() == (
        "32b8a2fb43edaa1a11d8d6df75c955c2f8e28d26efc29ee22fc5f2a2e323802e"
    )
    assert len(v5.canonical_json.encode()) == 663
    assert hashlib.sha256(v5.canonical_json.encode()).hexdigest() == (
        "0aaa56a0743044ea41ada3b4eaa775b7cb44256c2b0e65f092be760d856aafbe"
    )


def test_v5_automatically_demands_axis_state_and_scale_capabilities() -> None:
    world = _mixed_world(scale=2.0)
    requirements = {item.capability.value for item in world.requirements}
    assert "state.articulation.axis-units@1" in requirements
    assert "entity.scale.articulation.uniform@1" in requirements
    rigid = WorldSpec(
        "scaled-rigid",
        (
            EntitySpec(
                EntityPath("/box"),
                EntityKind.RIGID_BODY,
                box=BoxGeometrySpec(),
                scale_xyz=(2.0, 3.0, 4.0),
            ),
        ),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
    )
    rigid_requirements = {item.capability.value for item in rigid.requirements}
    assert "entity.scale.rigid@1" in rigid_requirements
    assert "state.rigid_body@1" in rigid_requirements


@pytest.mark.parametrize(
    "scale",
    (
        (0.0, 1.0, 1.0),
        (-1.0, 1.0, 1.0),
        (float("inf"), 1.0, 1.0),
        (1.0, 2.0, 1.0),
        (1, 1.0, 1.0),
    ),
)
def test_articulation_scale_rejects_invalid_nonuniform_and_nonexact_values(scale: tuple[object, ...]) -> None:
    with pytest.raises(ValidationError):
        EntitySpec(
            EntityPath("/robot"),
            EntityKind.ARTICULATION,
            joint_names=("joint",),
            scale_xyz=scale,  # type: ignore[arg-type]
        )


def test_v4_rejects_metre_axes_and_nonunit_scale() -> None:
    mixed = _mixed_world().entities[0]
    with pytest.raises(ValidationError, match="radian"):
        WorldSpec("legacy-mixed", (mixed,), schema_version=WORLD_SCHEMA_VERSION)
    scaled = replace(mixed, joint_position_units=("rad", "rad"), scale_xyz=(2.0, 2.0, 2.0))
    with pytest.raises(ValidationError, match="identity"):
        WorldSpec("legacy-scaled", (scaled,), schema_version=WORLD_SCHEMA_VERSION)


def test_provider_rejects_unsupported_schema_before_generation() -> None:
    descriptor = _descriptor_without(
        "control.articulation.position.axis-units@1",
        schemas=(WORLD_SCHEMA_VERSION,),
    )
    session = FakeSession(descriptor)
    with pytest.raises(UnsupportedCapabilityError) as failure:
        session.build(_mixed_world())
    assert failure.value.details["detail_code"] == WORLD_SCHEMA_UNSUPPORTED
    assert session._generation == 0


def test_mixed_state_is_self_closing_and_commands_use_selected_units() -> None:
    session = FakeSession(FAKE_DESCRIPTOR)
    world = session.build(_mixed_world())
    handle = world.resolve(EntityPath("/robot"))
    state = world.read_articulation(handle)
    assert state.entity_id == "/robot"
    assert state.generation == world.generation
    assert state.joint_names == ("hinge", "slide")
    assert state.joint_position_units == ("rad", "m")
    assert state.joint_velocity_units == ("rad/s", "m/s")

    world.apply_articulation_command(
        ArticulationCommand(
            handle,
            CommandMode.POSITION,
            ArrayValue.from_rows(((0.2, 0.4),)),
            degree_of_freedom_indices=(1, 0),
            target_units=("m", "rad"),
        )
    )
    world.step()
    assert world.read_articulation(handle).joint_positions.rows() == ((0.4, 0.2),)


@pytest.mark.parametrize(
    ("mode", "units"),
    (
        (CommandMode.POSITION, ("rad", "m")),
        (CommandMode.VELOCITY, ("rad/s", "m/s")),
        (CommandMode.EFFORT, ("N*m", "N")),
    ),
)
def test_each_command_mode_accepts_exact_mixed_units(mode: CommandMode, units: tuple[str, ...]) -> None:
    session = FakeSession(FAKE_DESCRIPTOR)
    world = session.build(_mixed_world())
    handle = world.resolve(EntityPath("/robot"))
    world.apply_articulation_command(
        ArticulationCommand(handle, mode, ArrayValue.from_rows(((0.1, 0.2),)), target_units=units)
    )


def test_wrong_units_fail_before_runtime_mutation() -> None:
    session = FakeSession(FAKE_DESCRIPTOR)
    world = session.build(_mixed_world())
    handle = world.resolve(EntityPath("/robot"))
    before = world.read_articulation(handle)
    with pytest.raises(CommandError) as failure:
        world.apply_articulation_command(
            ArticulationCommand(
                handle,
                CommandMode.POSITION,
                ArrayValue.from_rows(((0.3, 0.4),)),
                target_units=("m", "rad"),
            )
        )
    assert failure.value.details["detail_code"] == ARTICULATION_AXIS_UNITS_MISMATCH
    after = world.read_articulation(handle)
    assert after.joint_positions == before.joint_positions
    assert after.joint_velocities == before.joint_velocities


def test_missing_metre_position_capability_has_zero_state_side_effect() -> None:
    session = FakeSession(_descriptor_without("control.articulation.position.axis-units@1"))
    world = session.build(_mixed_world())
    handle = world.resolve(EntityPath("/robot"))
    before = world.read_articulation(handle)
    runtime = world._articulations[EntityPath("/robot")]
    modes_before = [row.copy() for row in runtime.modes]
    targets_before = [row.copy() for row in runtime.targets]
    with pytest.raises(UnsupportedCapabilityError) as failure:
        world.apply_articulation_command(
            ArticulationCommand(
                handle,
                CommandMode.POSITION,
                ArrayValue.from_rows(((0.3, 0.4),)),
                target_units=("rad", "m"),
            )
        )
    assert failure.value.details["detail_code"] == ARTICULATION_POSITION_AXIS_UNITS_UNSUPPORTED
    assert runtime.modes == modes_before
    assert runtime.targets == targets_before
    assert world.read_articulation(handle) == before


def test_all_radian_position_does_not_require_axis_units_capability() -> None:
    descriptor = _descriptor_without("control.articulation.position.axis-units@1")
    session = FakeSession(descriptor)
    spec = WorldSpec(
        "all-rad",
        (
            EntitySpec(
                EntityPath("/robot"),
                EntityKind.ARTICULATION,
                joint_names=("one", "two"),
                joint_position_units=("rad", "rad"),
            ),
        ),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
    )
    world = session.build(spec)
    handle = world.resolve(EntityPath("/robot"))
    world.apply_articulation_command(
        ArticulationCommand(
            handle,
            CommandMode.POSITION,
            ArrayValue.from_rows(((0.3, -0.2),)),
            target_units=("rad", "rad"),
        )
    )
    world.step()
    assert world.read_articulation(handle).joint_positions.rows() == ((0.3, -0.2),)


def test_missing_base_position_capability_fails_before_runtime_mutation() -> None:
    session = FakeSession(_descriptor_without("control.articulation.position@1"))
    world = session.build(_mixed_world())
    handle = world.resolve(EntityPath("/robot"))
    runtime = world._articulations[EntityPath("/robot")]
    modes_before = [row.copy() for row in runtime.modes]
    targets_before = [row.copy() for row in runtime.targets]
    with pytest.raises(UnsupportedCapabilityError) as failure:
        world.apply_articulation_command(
            ArticulationCommand(
                handle,
                CommandMode.POSITION,
                ArrayValue.from_rows(((0.3, 0.4),)),
                target_units=("rad", "m"),
            )
        )
    assert failure.value.details["capability_id"] == "control.articulation.position@1"
    assert runtime.modes == modes_before
    assert runtime.targets == targets_before


@pytest.mark.parametrize(
    ("capability", "entity"),
    (
        (
            "entity.scale.rigid@1",
            EntitySpec(
                EntityPath("/box"),
                EntityKind.RIGID_BODY,
                box=BoxGeometrySpec(),
                scale_xyz=(2.0, 3.0, 4.0),
            ),
        ),
        (
            "entity.scale.articulation.uniform@1",
            EntitySpec(
                EntityPath("/robot"),
                EntityKind.ARTICULATION,
                joint_names=("hinge",),
                scale_xyz=(2.0, 2.0, 2.0),
            ),
        ),
        (
            "state.articulation.axis-units@1",
            EntitySpec(
                EntityPath("/robot"),
                EntityKind.ARTICULATION,
                joint_names=("slide",),
                joint_position_units=("m",),
            ),
        ),
    ),
)
def test_optional_requirement_cannot_downgrade_automatic_physical_demand(
    capability: str,
    entity: EntitySpec,
) -> None:
    capability_id = CapabilityId(capability)
    descriptor = replace(
        FAKE_DESCRIPTOR,
        capabilities=CapabilitySet(
            tuple(item for item in FAKE_DESCRIPTOR.capabilities if item.capability != capability_id)
        ),
    )
    spec = WorldSpec(
        f"required-{capability.split('@', 1)[0].replace('.', '-')}",
        (entity,),
        requirements=(CapabilityRequirement(capability_id, required=False),),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
    )
    session = FakeSession(descriptor)
    with pytest.raises(CapabilityNegotiationError, match="requirements"):
        session.build(spec)
    assert session.side_effect_snapshot().generation == 0


def test_world_freezes_descriptor_and_exposes_zero_side_effect_counters() -> None:
    session = FakeSession(FAKE_DESCRIPTOR)
    world = session.build(_mixed_world())
    handle = world.resolve(EntityPath("/robot"))
    after_build = world.side_effect_snapshot()
    reduced = CapabilitySet(
        tuple(
            item
            for item in FAKE_DESCRIPTOR.capabilities
            if item.capability != CapabilityId("control.articulation.position@1")
        )
    )
    session._descriptor = replace(FAKE_DESCRIPTOR, capabilities=reduced)
    world.apply_articulation_command(
        ArticulationCommand(
            handle,
            CommandMode.POSITION,
            ArrayValue.from_rows(((0.1, 0.2),)),
            target_units=("rad", "m"),
        )
    )
    after_command = world.side_effect_snapshot()
    assert after_command.queues == after_build.queues + 1
    assert after_command.controller_lookups == after_build.controller_lookups + 1
    assert after_command.motor_enables == after_build.motor_enables + 1
    assert after_command.native_calls == after_build.native_calls + 1
    assert after_command.commands == after_build.commands + 1
    assert after_command.state_mutations == after_build.state_mutations + 1

    missing_axis = FakeSession(_descriptor_without("control.articulation.position.axis-units@1"))
    rejected_world = missing_axis.build(_mixed_world())
    rejected_handle = rejected_world.resolve(EntityPath("/robot"))
    before_rejection = rejected_world.side_effect_snapshot()
    with pytest.raises(UnsupportedCapabilityError):
        rejected_world.apply_articulation_command(
            ArticulationCommand(
                rejected_handle,
                CommandMode.POSITION,
                ArrayValue.from_rows(((0.1, 0.2),)),
                target_units=("rad", "m"),
            )
        )
    assert rejected_world.side_effect_snapshot() == before_rejection


def test_scaled_effective_planning_geometry_is_baked_with_identity_descriptor_scale() -> None:
    planning = CapabilityRequirement(CapabilityId("planning.scene@2"))
    spec = WorldSpec(
        "scaled-planning",
        (
            EntitySpec(
                EntityPath("/arm"),
                EntityKind.ARTICULATION,
                joint_names=("hinge",),
                scale_xyz=(2.0, 2.0, 2.0),
            ),
            EntitySpec(
                EntityPath("/box"),
                EntityKind.RIGID_BODY,
                box=BoxGeometrySpec(dimensions_m=(0.2, 0.3, 0.4)),
                scale_xyz=(2.0, 3.0, 4.0),
            ),
        ),
        requirements=(planning,),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
    )
    world = FakeProvider().open().build(spec)
    catalog = world.planning_scene_catalog()
    entity_by_path = {entity.path: entity for entity in catalog.entities}
    geometry_by_id = {geometry.geometry_id: geometry for geometry in catalog.geometries}

    box = geometry_by_id[entity_by_path["/box"].geometry_ids[0]]
    assert box.scale == (1.0, 1.0, 1.0)
    assert box.representation is PlanningGeometryRepresentation.BOX
    assert isinstance(box.inline, PlanningPrimitiveGeometry)
    assert box.inline.dimensions_m == pytest.approx((0.4, 0.9, 1.6))

    arm = geometry_by_id[entity_by_path["/arm"].geometry_ids[0]]
    assert arm.scale == (1.0, 1.0, 1.0)
    assert isinstance(arm.inline, PlanningCompoundGeometry)
    part_by_representation = {part.primitive.representation: part for part in arm.inline.parts}
    body = part_by_representation[PlanningGeometryRepresentation.BOX]
    column = part_by_representation[PlanningGeometryRepresentation.CYLINDER]
    assert body.local_pose.position_m == pytest.approx((0.08, -0.06, 0.16))
    assert body.primitive.dimensions_m == pytest.approx((0.6, 0.4, 0.32))
    assert column.local_pose.position_m == pytest.approx((-0.04, 0.1, 0.54))
    assert column.primitive.dimensions_m == pytest.approx((0.1, 0.7))


def test_v5_no_planning_demand_keeps_exact_plain_fake_world_shape() -> None:
    spec = WorldSpec(
        "physical-no-demand",
        (
            EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),
            EntitySpec(EntityPath("/arm"), EntityKind.ARTICULATION, joint_names=("joint",)),
        ),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
    )
    world = FakeProvider().open().build(spec)
    assert type(world) is FakeWorld
    assert not hasattr(world, "_planning_runtime")
    assert not any("planning" in name for name in world.__dict__)
    world.step(3)
    world.reset()
    assert not any("planning" in name for name in world.__dict__)
    world.close()


@pytest.mark.parametrize(
    "schemas",
    (
        [],
        (WORLD_SCHEMA_VERSION, WORLD_SCHEMA_VERSION),
        (PHYSICAL_WORLD_SCHEMA_VERSION, WORLD_SCHEMA_VERSION),
        ("bad-schema",),
    ),
)
def test_provider_descriptor_rejects_hostile_schema_declarations(schemas: object) -> None:
    with pytest.raises(ValidationError):
        ProviderDescriptor(
            "provider.test",
            "Provider",
            "1.0",
            "v0alpha5",
            CapabilitySet(()),
            schemas,  # type: ignore[arg-type]
            FrozenMap(),
        )


def test_fake_descriptor_declares_current_v4_v5_v6_world_schemas() -> None:
    assert FAKE_DESCRIPTOR.supported_world_schema_versions == (
        WORLD_SCHEMA_VERSION,
        PHYSICAL_WORLD_SCHEMA_VERSION,
        COMPOSITE_WORLD_SCHEMA_VERSION,
    )


def test_missing_state_axis_capability_fails_build_before_generation() -> None:
    session = FakeSession(_descriptor_without("state.articulation.axis-units@1"))
    with pytest.raises(Exception, match="requirements"):
        session.build(_mixed_world())
    assert session._generation == 0
