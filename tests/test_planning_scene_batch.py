from __future__ import annotations

import hashlib
import inspect
import threading

import pytest

import unirobosim.testing.fake_backend as fake_contract
from unirobosim import (
    CapabilityId,
    CapabilityRequirement,
    EntityKind,
    EntityPath,
    EntitySpec,
    FrozenMap,
    PlanningGeometryBatchWorld,
    PlanningGeometryRequest,
    PlanningGeometryResourceRevokedError,
    PlanningSceneContractError,
    PlanningSceneHashMismatchError,
    PlanningSceneNotFoundError,
    PlanningSceneWorld,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def _planning_requirement() -> CapabilityRequirement:
    return CapabilityRequirement(CapabilityId("planning.scene@2"))


@pytest.fixture
def batch_world():
    entities = tuple(
        EntitySpec(
            EntityPath(f"/mesh_{index}"),
            EntityKind.RIGID_BODY,
            asset_uri=f"asset://mesh-{index}",
            metadata=FrozenMap(
                {
                    "planning_motion_class": "static",
                    "fake_planning_collision_authority": "effective_native",
                }
            ),
        )
        for index in range(3)
    )
    spec = WorldSpec(
        "planning-batch-contract",
        entities,
        requirements=(_planning_requirement(),),
        metadata=FrozenMap(
            {
                "planning_attachment_authority": "exclusive_registry",
                "planning_attachments": (),
            }
        ),
    )
    session = FakeProvider().open()
    world = session.build(spec)
    try:
        yield world
    finally:
        session.close()


def _requests(world) -> tuple[PlanningGeometryRequest, ...]:
    return tuple(
        PlanningGeometryRequest(item.geometry_id)
        for item in world.planning_scene_catalog().geometries
        if item.resolution_key is not None
    )


def test_batch_protocol_is_additive_and_runtime_detectable(batch_world) -> None:
    assert isinstance(batch_world, PlanningSceneWorld)
    assert isinstance(batch_world, PlanningGeometryBatchWorld)
    assert "resolve_planning_geometries" not in PlanningSceneWorld.__dict__
    assert inspect.signature(PlanningGeometryBatchWorld.resolve_planning_geometries) == inspect.signature(
        type(batch_world).resolve_planning_geometries
    )


def test_batch_preserves_order_deduplicates_native_work_and_reuses_attested_cache(batch_world) -> None:
    requests = _requests(batch_world)
    requested = (requests[2], requests[0], requests[2], requests[1])
    requested_ids = tuple(item.geometry_id for item in requested)
    runtime = batch_world._planning_runtime

    lease = batch_world.resolve_planning_geometries(requested)
    assert tuple(item.geometry_id for item in lease.descriptors) == requested_ids
    assert lease.descriptors[0] is lease.descriptors[2]
    assert runtime.geometry_batch_resolves == 1
    assert runtime.geometry_materializations == 0
    assert runtime.geometry_hash_calls == 0
    assert runtime.storage_cache == {}

    first = lease.read(requests[0].geometry_id)
    assert hashlib.sha256(first).hexdigest() == lease.descriptors[1].sha256
    assert runtime.geometry_materializations == 1
    assert runtime.geometry_hash_calls == 1
    explicit = lease.read_many((requests[1].geometry_id, requests[0].geometry_id))
    assert tuple(identity for identity, _ in explicit) == (requests[1].geometry_id, requests[0].geometry_id)
    assert runtime.geometry_materializations == 2
    assert runtime.geometry_hash_calls == 2
    assert tuple(identity for identity, _ in lease.read_many()) == (
        requests[2].geometry_id,
        requests[0].geometry_id,
        requests[1].geometry_id,
    )
    assert runtime.geometry_materializations == 3
    assert runtime.geometry_hash_calls == 3
    lease.close()

    cached = batch_world.resolve_planning_geometries(requested)
    assert runtime.geometry_batch_resolves == 2
    assert runtime.geometry_materializations == 3
    assert runtime.geometry_hash_calls == 3
    cached.read_many()
    assert runtime.geometry_materializations == 3
    assert runtime.geometry_hash_calls == 3
    cached.close()


def test_batch_unknown_tail_fails_before_hash_cache_or_lease_publication(batch_world) -> None:
    request = _requests(batch_world)[0]
    runtime = batch_world._planning_runtime
    serial = runtime.environments[0].lease_serial
    with pytest.raises(PlanningSceneNotFoundError):
        batch_world.resolve_planning_geometries((request, PlanningGeometryRequest("geometry.unknown")))
    assert runtime.storage_cache == {}
    assert runtime.geometry_materializations == 0
    assert runtime.geometry_hash_calls == 0
    assert runtime.environments[0].lease_serial == serial


def test_batch_hash_failure_does_not_commit_earlier_materializations(batch_world) -> None:
    requests = _requests(batch_world)
    runtime = batch_world._planning_runtime
    environment = runtime.environments[0]
    corrupt_id = requests[1].geometry_id
    original = environment.raw_resources[corrupt_id]
    environment.raw_resources[corrupt_id] = (b"corrupt", *original[1:])
    try:
        lease = batch_world.resolve_planning_geometries(requests)
        with pytest.raises(PlanningSceneHashMismatchError, match="content hash"):
            lease.read_many()
        assert runtime.storage_cache == {}
        assert runtime.geometry_materializations == 0
        assert environment.lease_serial == 1
        lease.close()
    finally:
        environment.raw_resources[corrupt_id] = original


def test_batch_close_and_reset_revoke_all_reads_atomically(batch_world) -> None:
    requests = _requests(batch_world)
    lease = batch_world.resolve_planning_geometries(requests)
    lease.close()
    lease.close()
    assert lease.closed
    with pytest.raises(PlanningGeometryResourceRevokedError):
        _ = lease.descriptors
    with pytest.raises(PlanningGeometryResourceRevokedError):
        lease.read(requests[0].geometry_id)
    with pytest.raises(PlanningGeometryResourceRevokedError):
        lease.read_many()

    reset_lease = batch_world.resolve_planning_geometries(requests)
    batch_world.reset((0,))
    assert reset_lease.closed
    with pytest.raises(PlanningGeometryResourceRevokedError):
        reset_lease.read(requests[0].geometry_id)


def test_batch_read_is_worker_safe_bounded_and_rejects_unknown_ids(monkeypatch, batch_world) -> None:
    requests = _requests(batch_world)
    lease = batch_world.resolve_planning_geometries(requests)
    results: list[bytes] = []

    def worker() -> None:
        results.append(lease.read(requests[0].geometry_id, 0, 16))

    thread = threading.Thread(target=worker)
    thread.start()
    thread.join()
    assert len(results) == 1 and len(results[0]) == 16
    with pytest.raises(PlanningSceneNotFoundError):
        lease.read_many(("geometry.unknown",))
    monkeypatch.setattr(fake_contract, "PLANNING_GEOMETRY_BATCH_READ_LIMIT_BYTES", 1)
    with pytest.raises(PlanningSceneContractError, match="aggregate byte budget"):
        lease.read_many()
    lease.close()


def test_empty_batch_is_noop_and_single_api_uses_index_without_rehash(batch_world) -> None:
    empty = batch_world.resolve_planning_geometries(())
    assert empty.descriptors == ()
    assert empty.read_many() == ()
    empty.close()

    request = _requests(batch_world)[0]
    runtime = batch_world._planning_runtime
    first = batch_world.resolve_planning_geometry(request.geometry_id)
    second = batch_world.resolve_planning_geometry(request.geometry_id)
    assert runtime.geometry_single_resolves == 2
    assert runtime.geometry_materializations == 1
    assert runtime.geometry_hash_calls == 1
    first.close()
    second.close()
