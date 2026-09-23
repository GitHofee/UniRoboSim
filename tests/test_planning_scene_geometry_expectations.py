"""The validation expectation cache never caches acceptance of supplied state."""

import math
import random
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace

import pytest

from tests.test_planning_scene import planning_spec
from unirobosim import (
    PlanningGeometryLocalPose,
    PlanningPose,
    PlanningSceneContractError,
    PlanningSceneStaleGenerationError,
)
from unirobosim.api import planning_scene as contract
from unirobosim.testing import FakeProvider


def _unit(rng):
    values = [rng.uniform(-1., 1.) for _ in range(4)]
    norm = math.sqrt(sum(value * value for value in values))
    return tuple(value / norm for value in values)


def test_randomized_expected_pose_is_exactly_original_composition():
    rng = random.Random(20260911)
    for _ in range(1000):
        parent = PlanningPose('frame.world', tuple(rng.uniform(-100., 100.) for _ in range(3)), _unit(rng))
        local = PlanningGeometryLocalPose(tuple(rng.uniform(-100., 100.) for _ in range(3)), _unit(rng))
        expected = contract._compose_pose(parent, local)
        assert contract._geometry_expected_pose(parent, local) == expected
        assert contract._geometry_expected_pose(parent, local) == expected


@pytest.mark.parametrize('quaternion', [
    (0., 0., 0., -1.), (-1., 0., 0., 0.), (0., -1., 0., 0.),
    (0., 0., 0., 1. + .5e-6), (0., 0., 0., 1. - .5e-6),
    (0., 0., 0., 1. + 2.e-6), (0., 0., 0., 0.),
    (math.nan, 0., 0., 1.), (math.inf, 0., 0., 1.),
])
def test_canonicalization_and_invalid_mutated_quaternion_match_original(quaternion):
    parent = PlanningPose('frame.world', (0., 0., 0.), (0., 0., 0., 1.))
    local = PlanningGeometryLocalPose()
    contract._geometry_expected_pose(parent, local)
    object.__setattr__(parent, 'orientation_xyzw', quaternion)
    try:
        expected = contract._compose_pose(parent, local)
    except PlanningSceneContractError as failure:
        with pytest.raises(PlanningSceneContractError) as captured:
            contract._geometry_expected_pose(parent, local)
        assert str(captured.value) == str(failure)
        assert captured.value.operation == failure.operation
    else:
        assert contract._geometry_expected_pose(parent, local) == expected


@pytest.mark.parametrize('position', [(1.e308, 1.e308, 1.e308), (math.nan, 0., 0.), (math.inf, 0., 0.)])
def test_position_overflow_and_nonfinite_failures_match_original(position):
    parent = PlanningPose('frame.world', (0., 0., 0.), (0., 0., 0., 1.))
    local = PlanningGeometryLocalPose((1.e308, 1.e308, 1.e308))
    object.__setattr__(parent, 'position_m', position)
    with pytest.raises(PlanningSceneContractError) as original:
        contract._compose_pose(parent, local)
    with pytest.raises(PlanningSceneContractError) as cached:
        contract._geometry_expected_pose(parent, local)
    assert str(cached.value) == str(original.value)


def test_full_values_invalidate_cache_after_in_place_pose_or_local_edit():
    parent = PlanningPose('frame.world', (0., 0., 0.), (0., 0., 0., 1.))
    local = PlanningGeometryLocalPose()
    first = contract._geometry_expected_pose(parent, local)
    object.__setattr__(parent, 'position_m', (1., 2., 3.))
    moved = contract._geometry_expected_pose(parent, local)
    assert moved == contract._compose_pose(parent, local) and moved != first
    object.__setattr__(local, 'position_m', (4., 5., 6.))
    adjusted = contract._geometry_expected_pose(parent, local)
    assert adjusted == contract._compose_pose(parent, local) and adjusted != moved
    object.__setattr__(parent, 'frame_id', 'frame.other')
    assert contract._geometry_expected_pose(parent, local).frame_id == 'frame.other'
    object.__setattr__(parent, 'frame_id', 'invalid frame')
    with pytest.raises(PlanningSceneContractError):
        contract._geometry_expected_pose(parent, local)


def test_fallback_does_not_hash_hostile_scalar_subclasses():
    class UnhashableFloat(float):
        def __hash__(self):
            raise AssertionError('subclass hash must not be called')

    class UnhashableString(str):
        def __hash__(self):
            raise AssertionError('subclass hash must not be called')

    parent = PlanningPose('frame.world', (0., 0., 0.), (0., 0., 0., 1.))
    local = PlanningGeometryLocalPose()
    object.__setattr__(parent, 'position_m', (UnhashableFloat(1.), 0., 0.))
    object.__setattr__(parent, 'frame_id', UnhashableString('frame.world'))
    assert contract._geometry_expected_pose(parent, local) == contract._compose_pose(parent, local)


def test_warm_cache_still_rejects_contradictory_geometry_and_stale_generation():
    session = FakeProvider().open()
    try:
        world = session.build(planning_spec())
        catalog, state = world.planning_scene_catalog(), world.planning_scene_state()
        state.validate_against(catalog)
        first = state.geometry_transforms[0]
        position = first.world_pose.position_m
        within_tolerance = replace(
            first,
            world_pose=replace(first.world_pose, position_m=(position[0] + .5e-6, *position[1:])),
        )
        replace(state, geometry_transforms=(within_tolerance, *state.geometry_transforms[1:])).validate_against(catalog)
        changed = replace(first, world_pose=replace(first.world_pose, position_m=(1000., 0., 0.)))
        bad_state = replace(state, geometry_transforms=(changed, *state.geometry_transforms[1:]))
        with pytest.raises(PlanningSceneContractError, match='geometry world pose'):
            bad_state.validate_against(catalog)
        with pytest.raises(PlanningSceneStaleGenerationError):
            replace(state, generation=state.generation + 1).validate_against(catalog)
        state.validate_against(catalog)
    finally:
        session.close()


def test_cache_is_bounded_and_parallel_call_results_are_stable():
    contract._cached_geometry_expected_pose.cache_clear()
    parent = PlanningPose('frame.world', (0., 0., 0.), (0., 0., 0., 1.))
    local = PlanningGeometryLocalPose()
    for index in range(8300):
        contract._geometry_expected_pose(replace(parent, position_m=(float(index), 0., 0.)), local)
    assert contract._cached_geometry_expected_pose.cache_info().currsize == 8192

    def evaluate(index):
        pose = replace(parent, position_m=(float(index), 0., 0.))
        assert contract._geometry_expected_pose(pose, local) == contract._compose_pose(pose, local)

    with ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(evaluate, range(200)))
    assert contract._cached_geometry_expected_pose.cache_info().currsize <= 8192
    contract._cached_geometry_expected_pose.cache_clear()
