from __future__ import annotations

import math
import unittest

from tests.helpers import make_world_spec
from unirobosim import (
    ArrayValue,
    ArticulationCommand,
    CapabilityDeclaration,
    CapabilityId,
    CapabilityIssue,
    CapabilityRequirement,
    CapabilitySet,
    CommandMode,
    EntityKind,
    EntityPath,
    EntitySpec,
    FrozenMap,
    NegotiationReport,
    PhysicsSpec,
    ProviderRegistrationError,
    ProviderSelectionError,
    ValidationError,
    WorldSpec,
)
from unirobosim.runtime import ProviderRegistry
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider


class CapabilityValidationEdgeTests(unittest.TestCase):
    def test_requirement_and_declaration_wrong_types_are_structured(self) -> None:
        identifier = CapabilityId("test.feature@1")
        self.assertEqual(str(identifier), "test.feature@1")
        invalid_requirements = (
            ("test.feature@1", True, FrozenMap()),
            (identifier, "required", FrozenMap()),
            (identifier, True, {}),
        )
        for capability, required, constraints in invalid_requirements:
            with self.subTest(capability=capability), self.assertRaises(ValidationError):
                CapabilityRequirement(capability, required, constraints)  # type: ignore[arg-type]
        invalid_declarations = (
            ("test.feature@1", FrozenMap(), ()),
            (identifier, {}, ()),
            (identifier, FrozenMap(), None),
            (identifier, FrozenMap(), ("",)),
        )
        for capability, properties, limitations in invalid_declarations:
            with self.subTest(capability=capability), self.assertRaises(ValidationError):
                CapabilityDeclaration(capability, properties, limitations)  # type: ignore[arg-type]

    def test_issue_report_and_collection_edges_are_structured(self) -> None:
        identifier = CapabilityId("test.feature@1")
        for args in (
            ("bad", True, "missing", FrozenMap(), None),
            (identifier, True, "", FrozenMap(), None),
            (identifier, True, "missing", {}, None),
        ):
            with self.subTest(args=args), self.assertRaises(ValidationError):
                CapabilityIssue(*args)  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            NegotiationReport(("bad",), (), ())  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            NegotiationReport((), ("bad",), ())  # type: ignore[arg-type]
        capabilities = CapabilitySet((CapabilityDeclaration(identifier),))
        self.assertEqual(len(capabilities), 1)
        self.assertEqual(tuple(capabilities), (CapabilityDeclaration(identifier),))
        self.assertEqual(capabilities.get(identifier), CapabilityDeclaration(identifier))
        self.assertNotEqual(capabilities, object())
        self.assertIn("CapabilitySet", repr(capabilities))
        with self.assertRaises(ValidationError):
            capabilities.negotiate((object(),))  # type: ignore[arg-type]


class SpecificationValidationEdgeTests(unittest.TestCase):
    def test_physics_and_entity_conversion_errors_are_structured(self) -> None:
        for kwargs in (
            {"time_step_seconds": "bad"},
            {"gravity_m_s2": None},
            {"substeps": 0},
            {"gravity_m_s2": (0.0, 0.0)},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValidationError):
                PhysicsSpec(**kwargs)  # type: ignore[arg-type]

        path = EntityPath("/robot")
        invalid_entities = (
            {"path": "/robot", "kind": EntityKind.ARTICULATION, "joint_names": ("a",)},
            {"path": path, "kind": "articulation", "joint_names": ("a",)},
            {"path": path, "kind": EntityKind.ARTICULATION, "pose": object(), "joint_names": ("a",)},
            {"path": path, "kind": EntityKind.ARTICULATION, "joint_names": None},
            {
                "path": path,
                "kind": EntityKind.ARTICULATION,
                "joint_names": ("a",),
                "initial_joint_positions": (math.inf,),
            },
            {"path": path, "kind": EntityKind.ARTICULATION, "joint_names": ("a",), "asset_uri": ""},
            {"path": path, "kind": EntityKind.ARTICULATION, "joint_names": ("a",), "metadata": {}},
        )
        for kwargs in invalid_entities:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValidationError):
                EntitySpec(**kwargs)  # type: ignore[arg-type]

    def test_world_and_command_wrong_types_are_structured(self) -> None:
        robot = EntitySpec(EntityPath("/robot"), EntityKind.ARTICULATION, joint_names=("a",))
        with self.assertRaises(ValidationError):
            WorldSpec("world", (robot,), physics=object())  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            WorldSpec("world", None)  # type: ignore[arg-type]
        duplicate = CapabilityRequirement(CapabilityId("profile.core-robotics@1"))
        with self.assertRaises(ValidationError):
            WorldSpec("world", (robot,), requirements=(duplicate, duplicate))
        with self.assertRaises(ValidationError):
            WorldSpec("world", (robot,), metadata={})  # type: ignore[arg-type]

        session = FakeProvider().open()
        world = session.build(make_world_spec())
        handle = world.resolve(EntityPath("/robot"))
        try:
            invalid_commands = (
                (object(), CommandMode.POSITION, ArrayValue.from_rows(((0.0, 0.0),)), None),
                (handle, "position", ArrayValue.from_rows(((0.0, 0.0),)), None),
                (handle, CommandMode.POSITION, object(), None),
                (handle, CommandMode.POSITION, ArrayValue((2,), (0.0, 0.0)), None),
                (handle, CommandMode.POSITION, ArrayValue((1, 2), (1, 2), dtype="int64"), None),
                (handle, CommandMode.POSITION, ArrayValue.from_rows(((0.0, 0.0),)), 1),
                (handle, CommandMode.POSITION, ArrayValue.from_rows(((0.0, 0.0),)), (-1,)),
                (handle, CommandMode.POSITION, ArrayValue.from_rows(((0.0, 0.0),)), (0, 0)),
            )
            for candidate_handle, mode, targets, environments in invalid_commands:
                with self.subTest(mode=mode, environments=environments), self.assertRaises(ValidationError):
                    ArticulationCommand(
                        candidate_handle,
                        mode,
                        targets,
                        environment_indices=environments,
                    )  # type: ignore[arg-type]
        finally:
            session.close()


class RegistryValidationEdgeTests(unittest.TestCase):
    def test_registration_and_factory_failures_are_structured(self) -> None:
        registry = ProviderRegistry()
        with self.assertRaises(ProviderRegistrationError):
            registry.register(FAKE_DESCRIPTOR, None)  # type: ignore[arg-type]

        def failed_factory():
            raise RuntimeError("factory failed")

        registry.register(FAKE_DESCRIPTOR, failed_factory)
        with self.assertRaises(ProviderSelectionError) as caught:
            registry.create("reference.fake")
        self.assertIsInstance(caught.exception.__cause__, RuntimeError)

    def test_factory_protocol_and_descriptor_mismatch_are_rejected(self) -> None:
        registry = ProviderRegistry()
        registry.register(FAKE_DESCRIPTOR, lambda: object())  # type: ignore[arg-type]
        with self.assertRaises(ProviderRegistrationError):
            registry.create("reference.fake")

        class DifferentDescriptorProvider(FakeProvider):
            @property
            def descriptor(self):
                return type(FAKE_DESCRIPTOR)(
                    "reference.different",
                    FAKE_DESCRIPTOR.display_name,
                    FAKE_DESCRIPTOR.version,
                    FAKE_DESCRIPTOR.contract_version,
                    FAKE_DESCRIPTOR.capabilities,
                )

        mismatch = ProviderRegistry()
        mismatch.register(FAKE_DESCRIPTOR, DifferentDescriptorProvider)
        with self.assertRaises(ProviderRegistrationError):
            mismatch.create("reference.fake")


if __name__ == "__main__":
    unittest.main()
