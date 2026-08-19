from __future__ import annotations

import hashlib
from dataclasses import replace
from pathlib import Path
from typing import Any

import pytest

from unirobosim import (
    AssetNormalizationError,
    AssetNormalizationInspection,
    AssetNormalizationRequest,
    AssetNormalizationResult,
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
from unirobosim.easy import normalization as normalization_module
from unirobosim.easy.normalization import discover_asset_normalizers, select_asset_normalizer
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider


class NormalizationAwareFakeProvider(FakeProvider):
    @property
    def descriptor(self) -> ProviderDescriptor:
        declarations = (
            CapabilityDeclaration(
                CapabilityId("asset.formats@1"),
                FrozenMap(
                    {
                        "rigid_body": ["model/vnd.usd"],
                        "articulation": ["model/vnd.usd"],
                    }
                ),
            ),
            CapabilityDeclaration(
                CapabilityId("asset.normalization@1"),
                FrozenMap(
                    {
                        "rigid_body": {
                            "media_type": "model/vnd.usd",
                            "profile": "test.dynamic-rigid-usd@1",
                        }
                    }
                ),
            ),
        )
        return ProviderDescriptor(
            FAKE_DESCRIPTOR.provider_id,
            FAKE_DESCRIPTOR.display_name,
            FAKE_DESCRIPTOR.version,
            FAKE_DESCRIPTOR.contract_version,
            CapabilitySet((*FAKE_DESCRIPTOR.capabilities, *declarations)),
            FAKE_DESCRIPTOR.metadata,
        )


class RecordingNormalizer:
    normalizer_id = "test.usd-physics"

    def __init__(self, output: Path, *, required: bool = True) -> None:
        self.output = output
        self.required = required
        self.requests: list[AssetNormalizationRequest] = []
        self.normalizations = 0

    def can_normalize(self, request: AssetNormalizationRequest) -> bool:
        return (
            request.source_media_type == "model/vnd.usd"
            and request.entity_kind is EntityKind.RIGID_BODY
            and request.target_profile == "test.dynamic-rigid-usd@1"
        )

    def inspect(self, request: AssetNormalizationRequest) -> AssetNormalizationInspection:
        self.requests.append(request)
        return AssetNormalizationInspection(
            self.required,
            "render-only" if self.required else "ready",
            ("missing rigid-body API",) if self.required else (),
            {"mesh_count": 1},
        )

    def normalize(
        self,
        request: AssetNormalizationRequest,
        inspection: AssetNormalizationInspection,
    ) -> AssetNormalizationResult:
        self.normalizations += 1
        self.output.write_text("#usda 1.0\n", encoding="utf-8")
        output_hash = hashlib.sha256(self.output.read_bytes()).hexdigest()
        return AssetNormalizationResult(
            uri=str(self.output),
            media_type="model/vnd.usd",
            normalizer_id=self.normalizer_id,
            normalizer_version="1.0.0",
            target_profile=request.target_profile,
            source_sha256="1" * 64,
            recipe_sha256="2" * 64,
            output_sha256=output_hash,
            report_uri=str(self.output.with_suffix(".json")),
            inspection=inspection,
            warnings=("test normalization",),
        )


def _request(tmp_path: Path) -> AssetNormalizationRequest:
    return AssetNormalizationRequest(
        "source.usd",
        "model/vnd.usd",
        "isaaclab",
        "nvidia.isaaclab",
        EntityKind.RIGID_BODY,
        "test.dynamic-rigid-usd@1",
        str(tmp_path),
        {"collision_mode": "auto"},
    )


def _result(request: AssetNormalizationRequest) -> AssetNormalizationResult:
    inspection = AssetNormalizationInspection(True, "needs-normalization", ("missing physics",), {"meshes": 1})
    return AssetNormalizationResult(
        "normalized.usd",
        "model/vnd.usd",
        "test.normalizer",
        "1.0.0",
        request.target_profile,
        "1" * 64,
        "2" * 64,
        "3" * 64,
        "normalization.json",
        inspection,
        warnings=("derived collision",),
    )


def test_supported_usd_is_semantically_normalized(tmp_path: Path) -> None:
    source = tmp_path / "visual-only.usd"
    source.write_text("#usda 1.0\n", encoding="utf-8")
    normalizer = RecordingNormalizer(tmp_path / "normalized.usd")
    sim = Sim(
        provider=NormalizationAwareFakeProvider(),
        asset_normalizers=(normalizer,),
        asset_cache_directory=tmp_path / "cache",
    )
    sim.add_rigid_body(
        "cup",
        asset_uri=str(source),
        asset_options={"preserve_cavities": True, "collision_mode": "auto"},
    )

    sim.start()

    request = normalizer.requests[0]
    assert request.options["preserve_cavities"] is True
    assert request.target_profile == "test.dynamic-rigid-usd@1"
    assert sim.world_spec.entities[0].asset_uri == str(normalizer.output)
    metadata = sim.world_spec.entities[0].metadata["unirobosim_asset"]
    assert metadata["selector"] == "normalized:test.usd-physics"
    assert metadata["normalization"]["inspection"]["classification"] == "render-only"
    sim.close()


def test_ready_usd_is_not_rewritten_and_prebuilt_policy_skips_plugin(tmp_path: Path) -> None:
    source = tmp_path / "ready.usd"
    source.write_text("#usda 1.0\n", encoding="utf-8")
    ready = RecordingNormalizer(tmp_path / "unused.usd", required=False)
    sim = Sim(provider=NormalizationAwareFakeProvider(), asset_normalizers=(ready,))
    sim.add_rigid_body("ready", asset_uri=str(source))
    sim.start()
    assert sim.world_spec.entities[0].asset_uri == str(source)
    assert ready.normalizations == 0
    sim.close()

    strict = RecordingNormalizer(tmp_path / "also-unused.usd")
    sim = Sim(
        provider=NormalizationAwareFakeProvider(),
        asset_policy=AssetPolicy.PREBUILT_ONLY,
        asset_normalizers=(strict,),
    )
    sim.add_rigid_body("ready", asset_uri=str(source))
    sim.start()
    assert strict.requests == []
    sim.close()


def test_incompatible_normalizer_result_and_invalid_declaration_fail(tmp_path: Path) -> None:
    source = tmp_path / "visual-only.usd"
    source.write_text("#usda 1.0\n", encoding="utf-8")
    normalizer = RecordingNormalizer(tmp_path / "normalized.usd")
    original = normalizer.normalize

    def wrong_profile(
        request: AssetNormalizationRequest,
        inspection: AssetNormalizationInspection,
    ) -> AssetNormalizationResult:
        result = original(request, inspection)
        return replace(result, target_profile="wrong@1")

    normalizer.normalize = wrong_profile  # type: ignore[method-assign]
    sim = Sim(provider=NormalizationAwareFakeProvider(), asset_normalizers=(normalizer,))
    sim.add_rigid_body("object", asset_uri=str(source))
    with pytest.raises(AssetNormalizationError, match="incompatible target"):
        sim.start()


def test_normalization_contract_validation_and_option_alias(tmp_path: Path) -> None:
    with pytest.raises(ValidationError):
        AssetNormalizationRequest(
            "source.usd",
            "invalid",
            "isaaclab",
            "nvidia.isaaclab",
            EntityKind.RIGID_BODY,
            "isaaclab.dynamic-rigid-usd@1",
            str(tmp_path),
        )
    with pytest.raises(ValidationError):
        AssetNormalizationInspection(True, "")

    request = _request(tmp_path)
    assert request.to_dict()["options"] == {"collision_mode": "auto"}
    inspection = AssetNormalizationInspection(True, "needs-normalization", ["missing physics"], {"mesh_count": 1})
    assert inspection.to_dict()["reasons"] == ["missing physics"]
    result = _result(request)
    assert result.to_dict()["inspection"]["classification"] == inspection.classification
    assert result.to_dict()["warnings"] == ["derived collision"]

    sim = Sim(provider=NormalizationAwareFakeProvider(), asset_normalizers=())
    with pytest.raises(ValidationError, match="mutually exclusive"):
        sim.add_rigid_body(
            "object",
            asset_uri="object.usd",
            asset_options={"mass_kg": 1.0},
            conversion_options={"mass_kg": 1.0},
        )


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("source_uri", ""),
        ("source_media_type", "usd"),
        ("target_backend", ""),
        ("provider_id", ""),
        ("entity_kind", "rigid_body"),
        ("target_profile", ""),
        ("cache_directory", ""),
    ),
)
def test_normalization_request_rejects_invalid_fields(tmp_path: Path, field: str, value: object) -> None:
    values: dict[str, object] = {
        "source_uri": "source.usd",
        "source_media_type": "model/vnd.usd",
        "target_backend": "isaaclab",
        "provider_id": "nvidia.isaaclab",
        "entity_kind": EntityKind.RIGID_BODY,
        "target_profile": "isaaclab.dynamic-rigid-usd@1",
        "cache_directory": str(tmp_path),
    }
    values[field] = value
    with pytest.raises(ValidationError):
        AssetNormalizationRequest(**values)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "arguments",
    (
        (1, "ready", (), {}),
        (True, "", (), {}),
        (True, "ready", ("",), {}),
    ),
)
def test_normalization_inspection_rejects_invalid_fields(arguments: tuple[Any, ...]) -> None:
    with pytest.raises(ValidationError):
        AssetNormalizationInspection(*arguments)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("uri", ""),
        ("media_type", "usd"),
        ("source_sha256", "not-a-digest"),
        ("recipe_sha256", "a" * 63),
        ("output_sha256", "z" * 64),
        ("inspection", object()),
        ("cache_hit", "yes"),
        ("warnings", ("",)),
    ),
)
def test_normalization_result_rejects_invalid_fields(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    request = _request(tmp_path)
    result = _result(request)
    with pytest.raises(ValidationError):
        replace(result, **{field: value})


class _EntryPoint:
    def __init__(self, name: str, factory: object) -> None:
        self.name = name
        self._factory = factory

    def load(self) -> object:
        if isinstance(self._factory, Exception):
            raise self._factory
        return self._factory


def test_normalizer_discovery_is_deterministic_and_tolerates_partial_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    accepted = RecordingNormalizer(tmp_path / "normalized.usd")
    points = (
        _EntryPoint("z-broken", RuntimeError("broken plugin")),
        _EntryPoint("a-valid", lambda: accepted),
    )
    monkeypatch.setattr(normalization_module.metadata, "entry_points", lambda **_: points)
    assert discover_asset_normalizers() == (accepted,)

    monkeypatch.setattr(
        normalization_module.metadata,
        "entry_points",
        lambda **_: (_EntryPoint("invalid", lambda: object()),),
    )
    with pytest.raises(AssetNormalizationError, match="could not be loaded") as failure:
        discover_asset_normalizers()
    assert failure.value.details["failures"][0]["entry_point"] == "invalid"


def test_normalizer_selection_records_rejections_and_plugin_errors(tmp_path: Path) -> None:
    request = _request(tmp_path)

    class Rejecting(RecordingNormalizer):
        normalizer_id = "test.rejecting"

        def can_normalize(self, request: AssetNormalizationRequest) -> bool:
            return False

    class Crashing(RecordingNormalizer):
        normalizer_id = "test.crashing"

        def can_normalize(self, request: AssetNormalizationRequest) -> bool:
            raise RuntimeError("probe failed")

    rejecting = Rejecting(tmp_path / "rejected.usd")
    crashing = Crashing(tmp_path / "crashed.usd")
    accepted = RecordingNormalizer(tmp_path / "accepted.usd")
    assert select_asset_normalizer(request, (crashing, rejecting, accepted)) is accepted
    with pytest.raises(AssetNormalizationError, match="no installed") as failure:
        select_asset_normalizer(request, (crashing, rejecting))
    attempts = failure.value.details["attempts"]
    assert attempts[0]["error"] == "RuntimeError: probe failed"
    assert attempts[1]["accepted"] is False


def test_sim_rejects_invalid_normalizer_and_wraps_unexpected_failures(tmp_path: Path) -> None:
    with pytest.raises(ValidationError, match="asset_normalizers"):
        Sim(provider=NormalizationAwareFakeProvider(), asset_normalizers=(object(),))  # type: ignore[arg-type]

    source = tmp_path / "visual.usd"
    source.write_text("#usda 1.0\n", encoding="utf-8")
    crashing = RecordingNormalizer(tmp_path / "never.usd")

    def fail_inspection(_: AssetNormalizationRequest) -> AssetNormalizationInspection:
        raise RuntimeError("inspection crashed")

    crashing.inspect = fail_inspection  # type: ignore[method-assign]
    sim = Sim(provider=NormalizationAwareFakeProvider(), asset_normalizers=(crashing,))
    sim.add_rigid_body("object", asset_uri=str(source))
    with pytest.raises(AssetNormalizationError, match="asset normalizer failed") as failure:
        sim.start()
    assert isinstance(failure.value.__cause__, RuntimeError)


def test_provider_normalization_declaration_is_strict(tmp_path: Path) -> None:
    source = tmp_path / "source.usd"
    source.write_text("#usda 1.0\n", encoding="utf-8")

    class InvalidProvider(NormalizationAwareFakeProvider):
        def __init__(self, value: object) -> None:
            super().__init__()
            self.value = value

        @property
        def descriptor(self) -> ProviderDescriptor:
            declaration = CapabilityDeclaration(
                CapabilityId("asset.normalization@1"),
                FrozenMap({"rigid_body": self.value}),
            )
            formats = CapabilityDeclaration(
                CapabilityId("asset.formats@1"),
                FrozenMap({"rigid_body": ["model/vnd.usd"]}),
            )
            return ProviderDescriptor(
                FAKE_DESCRIPTOR.provider_id,
                FAKE_DESCRIPTOR.display_name,
                FAKE_DESCRIPTOR.version,
                FAKE_DESCRIPTOR.contract_version,
                CapabilitySet((*FAKE_DESCRIPTOR.capabilities, formats, declaration)),
                FAKE_DESCRIPTOR.metadata,
            )

    for declaration in ("invalid", {"media_type": "usd", "profile": ""}):
        sim = Sim(provider=InvalidProvider(declaration), asset_normalizers=())
        sim.add_rigid_body("object", asset_uri=str(source))
        with pytest.raises(AssetNormalizationError, match="declaration|target"):
            sim.start()


def test_no_normalizer_keeps_provider_native_asset_and_articulation_options(tmp_path: Path) -> None:
    source = tmp_path / "source.usd"
    source.write_text("#usda 1.0\n", encoding="utf-8")
    sim = Sim(provider=NormalizationAwareFakeProvider(), asset_normalizers=())
    sim.add_articulation(
        "cabinet",
        joint_names=("door",),
        asset_uri=str(source),
        asset_options={"collision_mode": "convex_decomposition"},
    )
    sim.start()
    assert sim.world_spec.entities[0].asset_uri == str(source)
    sim.close()
