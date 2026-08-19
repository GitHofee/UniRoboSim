from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from unirobosim import (
    AssetBundle,
    AssetConversionError,
    AssetConversionRequest,
    AssetConversionResult,
    AssetPolicy,
    CapabilityDeclaration,
    CapabilityId,
    CapabilitySet,
    EntityKind,
    FrozenMap,
    ProviderDescriptor,
    Sim,
    ValidationError,
)
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider


class FormatAwareFakeProvider(FakeProvider):
    @property
    def descriptor(self) -> ProviderDescriptor:
        formats = CapabilityDeclaration(
            CapabilityId("asset.formats@1"),
            FrozenMap(
                {
                    "rigid_body": ["model/vnd.urdf+xml"],
                    "articulation": ["model/vnd.urdf+xml"],
                }
            ),
        )
        return ProviderDescriptor(
            FAKE_DESCRIPTOR.provider_id,
            FAKE_DESCRIPTOR.display_name,
            FAKE_DESCRIPTOR.version,
            FAKE_DESCRIPTOR.contract_version,
            CapabilitySet((*FAKE_DESCRIPTOR.capabilities, formats)),
            FAKE_DESCRIPTOR.metadata,
        )


class RecordingConverter:
    converter_id = "test.usd-rigid"

    def __init__(self, output: Path) -> None:
        self.output = output
        self.requests: list[AssetConversionRequest] = []

    def can_convert(self, request: AssetConversionRequest) -> bool:
        return request.source_media_type == "model/vnd.usd" and request.entity_kind is EntityKind.RIGID_BODY

    def convert(self, request: AssetConversionRequest) -> AssetConversionResult:
        self.requests.append(request)
        self.output.write_text("<robot name='converted'/>", encoding="utf-8")
        output_sha = hashlib.sha256(self.output.read_bytes()).hexdigest()
        return AssetConversionResult(
            uri=str(self.output),
            media_type="model/vnd.urdf+xml",
            converter_id=self.converter_id,
            converter_version="1.0.0",
            source_sha256="1" * 64,
            recipe_sha256="2" * 64,
            output_sha256=output_sha,
            report_uri=str(self.output.with_suffix(".json")),
            warnings=("test approximation",),
        )


def test_direct_usd_is_converted_after_provider_selection(tmp_path: Path) -> None:
    source = tmp_path / "object.usd"
    source.write_text("#usda 1.0", encoding="utf-8")
    converter = RecordingConverter(tmp_path / "object.urdf")
    sim = Sim(
        provider=FormatAwareFakeProvider(),
        asset_converters=(converter,),
        asset_cache_directory=tmp_path / "cache",
    )
    sim.add_rigid_body(
        "object",
        asset_uri=str(source),
        conversion_options={"mass_kg": 0.25},
    )

    sim.start()

    request = converter.requests[0]
    assert request.provider_id == "reference.fake"
    assert request.options["mass_kg"] == 0.25
    assert sim.world_spec.entities[0].asset_uri == str(converter.output)
    metadata = sim.world_spec.entities[0].metadata["unirobosim_asset"]
    assert metadata["selector"] == "converted:test.usd-rigid"
    assert metadata["conversion"]["warnings"] == ("test approximation",)
    sim.close()


def test_asset_bundle_uses_canonical_usd_when_native_variant_is_absent(tmp_path: Path) -> None:
    source = tmp_path / "object.usdc"
    source.write_bytes(b"usd")
    converter = RecordingConverter(tmp_path / "object.urdf")
    sim = Sim(provider=FormatAwareFakeProvider(), asset_converters=(converter,))
    sim.add_rigid_body("object", asset=AssetBundle("object", {"source": str(source)}))

    sim.start()

    assert converter.requests[0].source_uri == str(source)
    assert sim.world_spec.entities[0].metadata["unirobosim_asset"].get("source_manifest") is None
    sim.close()


def test_prebuilt_policy_and_bad_converter_format_fail_actionably(tmp_path: Path) -> None:
    source = tmp_path / "object.usd"
    source.write_text("#usda 1.0", encoding="utf-8")
    strict = Sim(provider=FormatAwareFakeProvider(), asset_policy=AssetPolicy.PREBUILT_ONLY)
    strict.add_rigid_body("object", asset_uri=str(source))
    with pytest.raises(AssetConversionError, match="conversion is disabled"):
        strict.start()

    converter = RecordingConverter(tmp_path / "object.urdf")
    sim = Sim(provider=FormatAwareFakeProvider(), asset_converters=(converter,))
    sim.add_rigid_body("object", asset_uri=str(source))
    converter_output = converter.convert

    def wrong_format(request: AssetConversionRequest) -> AssetConversionResult:
        result = converter_output(request)
        return AssetConversionResult(**{**result.to_dict(), "media_type": "application/x-unsupported"})

    converter.convert = wrong_format  # type: ignore[method-assign]
    with pytest.raises(AssetConversionError, match="unsupported by the provider"):
        sim.start()


@pytest.mark.parametrize("value", ("", "not-a-media-type"))
def test_conversion_request_rejects_invalid_media_type(value: str) -> None:
    with pytest.raises(ValidationError):
        AssetConversionRequest(
            "source.usd",
            value,
            "fake",
            "reference.fake",
            EntityKind.RIGID_BODY,
            "/tmp/cache",
        )
