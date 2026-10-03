"""Actual immutable solver topology, after any backend meshing."""

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from .capabilities import CapabilityId
from .specs import DeformableBodySpec
from .values import ArrayValue, DeformableTopology, EntityHandle

DEFORMABLE_TOPOLOGY_CAPABILITY_ID = CapabilityId("deformable.topology.read@1")


@dataclass(frozen=True)
class DeformableTopologySnapshot:
    """Entity-local rest nodes and connectivity in read_deformable node order.

    Node indices remain stable for the built world's generation. Topology reads
    must reject stale handles and never advance the physics clock.
    """

    topology: DeformableTopology
    rest_positions_m: ArrayValue
    surface_triangles: ArrayValue | None = None
    tetrahedra: ArrayValue | None = None

    def __post_init__(self) -> None:
        DeformableBodySpec(self.topology, self.rest_positions_m, self.surface_triangles, self.tetrahedra)

    @property
    def node_count(self) -> int:
        return self.rest_positions_m.shape[0]


@runtime_checkable
class DeformableTopologyWorld(Protocol):
    def read_deformable_topology(self, handle: EntityHandle) -> DeformableTopologySnapshot: ...
