from __future__ import annotations

import unittest

from unirobosim import (
    ArrayValue,
    ArticulationState,
    BuildFingerprint,
    BuildReport,
    CapabilitySet,
    ProbeReport,
    ProviderDescriptor,
    ResetResult,
    Tick,
    ValidationError,
)
from unirobosim.testing import FAKE_DESCRIPTOR


class ReportValidationTests(unittest.TestCase):
    def test_provider_and_probe_validation(self) -> None:
        descriptor = ProviderDescriptor("valid.provider", "Valid", "1", "v1", CapabilitySet())
        self.assertEqual(descriptor.provider_id, "valid.provider")
        for args in (
            ("Invalid Provider", "Valid", "1", "v1", CapabilitySet()),
            ("valid", "", "1", "v1", CapabilitySet()),
            ("valid", "Valid", "1", "v1", object()),
        ):
            with self.subTest(args=args), self.assertRaises(ValidationError):
                ProviderDescriptor(*args)  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            ProbeReport(FAKE_DESCRIPTOR, True, details={})  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            ProbeReport(FAKE_DESCRIPTOR, "yes")  # type: ignore[arg-type]

    def test_build_fingerprint_and_report_validation(self) -> None:
        digest = "a" * 64
        fingerprint = BuildFingerprint("provider", "1", "v1", digest, digest)
        report = BuildReport(fingerprint, "world", 1, 2, 3)
        self.assertEqual(report.fingerprint.to_dict()["world_digest"], digest)
        with self.assertRaises(ValidationError):
            BuildFingerprint("provider", "1", "v1", "bad", digest)
        with self.assertRaises(ValidationError):
            BuildReport(fingerprint, "world", 0, 2, 3)

    def test_reset_and_articulation_state_validation(self) -> None:
        tick = Tick(0, 0.0)
        reset = ResetResult((1, 0), 1, tick)
        self.assertEqual(reset.environment_indices, (1, 0))
        with self.assertRaises(ValidationError):
            ResetResult((0, 0), 1, tick)
        with self.assertRaises(ValidationError):
            ResetResult((0,), 0, tick)
        positions = ArrayValue.from_rows(((0.0, 1.0),))
        velocities = ArrayValue.from_rows(((0.0,),))
        with self.assertRaises(ValidationError):
            ArticulationState(positions, velocities, tick)
        with self.assertRaises(ValidationError):
            ArticulationState(positions, positions, object())  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
