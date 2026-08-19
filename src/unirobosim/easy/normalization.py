"""Provider-declared semantic normalization contracts for EasyAPI assets."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from importlib import metadata
from typing import Protocol, runtime_checkable

from unirobosim.api.errors import AssetNormalizationError, ValidationError
from unirobosim.api.frozen import FrozenMap
from unirobosim.api.values import EntityKind


def _digest(name: str, value: str, operation: str) -> None:
    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value.lower()):
        raise ValidationError(f"{name} must be a SHA-256 digest", operation=operation)


@dataclass(frozen=True)
class AssetNormalizationRequest:
    source_uri: str
    source_media_type: str
    target_backend: str
    provider_id: str
    entity_kind: EntityKind
    target_profile: str
    cache_directory: str
    options: FrozenMap | Mapping[str, object] = field(default_factory=FrozenMap)

    def __post_init__(self) -> None:
        operation = "asset.normalize.validate"
        string_fields = (
            self.source_uri,
            self.source_media_type,
            self.target_backend,
            self.provider_id,
            self.target_profile,
            self.cache_directory,
        )
        if any(not isinstance(value, str) or not value.strip() for value in string_fields):
            raise ValidationError("normalization request fields must be non-empty strings", operation=operation)
        if "/" not in self.source_media_type:
            raise ValidationError("normalization source media type is invalid", operation=operation)
        if not isinstance(self.entity_kind, EntityKind):
            raise ValidationError("normalization entity kind is invalid", operation=operation)
        object.__setattr__(
            self,
            "options",
            self.options if isinstance(self.options, FrozenMap) else FrozenMap(self.options),
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "source_uri": self.source_uri,
            "source_media_type": self.source_media_type,
            "target_backend": self.target_backend,
            "provider_id": self.provider_id,
            "entity_kind": self.entity_kind.value,
            "target_profile": self.target_profile,
            "cache_directory": self.cache_directory,
            "options": FrozenMap(self.options).to_dict(),
        }


@dataclass(frozen=True)
class AssetNormalizationInspection:
    required: bool
    classification: str
    reasons: tuple[str, ...] = ()
    details: FrozenMap | Mapping[str, object] = field(default_factory=FrozenMap)

    def __post_init__(self) -> None:
        if not isinstance(self.required, bool):
            raise ValidationError("normalization required flag must be boolean", operation="asset.normalize.inspect")
        if not isinstance(self.classification, str) or not self.classification.strip():
            raise ValidationError("normalization classification must be non-empty", operation="asset.normalize.inspect")
        reasons = tuple(self.reasons)
        if any(not isinstance(reason, str) or not reason.strip() for reason in reasons):
            raise ValidationError("normalization reasons are invalid", operation="asset.normalize.inspect")
        object.__setattr__(self, "reasons", reasons)
        object.__setattr__(
            self,
            "details",
            self.details if isinstance(self.details, FrozenMap) else FrozenMap(self.details),
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "required": self.required,
            "classification": self.classification,
            "reasons": list(self.reasons),
            "details": FrozenMap(self.details).to_dict(),
        }


@dataclass(frozen=True)
class AssetNormalizationResult:
    uri: str
    media_type: str
    normalizer_id: str
    normalizer_version: str
    target_profile: str
    source_sha256: str
    recipe_sha256: str
    output_sha256: str
    report_uri: str
    inspection: AssetNormalizationInspection
    cache_hit: bool = False
    warnings: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        operation = "asset.normalize.result"
        strings = (
            self.uri,
            self.media_type,
            self.normalizer_id,
            self.normalizer_version,
            self.target_profile,
            self.source_sha256,
            self.recipe_sha256,
            self.output_sha256,
            self.report_uri,
        )
        if any(not isinstance(value, str) or not value.strip() for value in strings):
            raise ValidationError("normalization result fields must be non-empty strings", operation=operation)
        if "/" not in self.media_type:
            raise ValidationError("normalization result media type is invalid", operation=operation)
        _digest("source_sha256", self.source_sha256, operation)
        _digest("recipe_sha256", self.recipe_sha256, operation)
        _digest("output_sha256", self.output_sha256, operation)
        if not isinstance(self.inspection, AssetNormalizationInspection):
            raise ValidationError("normalization result inspection is invalid", operation=operation)
        warnings = tuple(self.warnings)
        if not isinstance(self.cache_hit, bool) or any(
            not isinstance(warning, str) or not warning.strip() for warning in warnings
        ):
            raise ValidationError("normalization result cache/warnings are invalid", operation=operation)
        object.__setattr__(self, "warnings", warnings)

    def to_dict(self) -> dict[str, object]:
        return {
            "uri": self.uri,
            "media_type": self.media_type,
            "normalizer_id": self.normalizer_id,
            "normalizer_version": self.normalizer_version,
            "target_profile": self.target_profile,
            "source_sha256": self.source_sha256,
            "recipe_sha256": self.recipe_sha256,
            "output_sha256": self.output_sha256,
            "report_uri": self.report_uri,
            "inspection": self.inspection.to_dict(),
            "cache_hit": self.cache_hit,
            "warnings": list(self.warnings),
        }


@runtime_checkable
class AssetNormalizer(Protocol):
    @property
    def normalizer_id(self) -> str: ...

    def can_normalize(self, request: AssetNormalizationRequest) -> bool: ...

    def inspect(self, request: AssetNormalizationRequest) -> AssetNormalizationInspection: ...

    def normalize(
        self,
        request: AssetNormalizationRequest,
        inspection: AssetNormalizationInspection,
    ) -> AssetNormalizationResult: ...


def discover_asset_normalizers() -> tuple[AssetNormalizer, ...]:
    normalizers: list[AssetNormalizer] = []
    failures: list[dict[str, str]] = []
    points = tuple(sorted(metadata.entry_points(group="unirobosim.asset_normalizers"), key=lambda item: item.name))
    for point in points:
        try:
            normalizer = point.load()()
            if not isinstance(normalizer, AssetNormalizer):
                raise TypeError("entry point did not return an AssetNormalizer")
            normalizers.append(normalizer)
        except Exception as exc:
            failures.append({"entry_point": point.name, "error": f"{type(exc).__name__}: {exc}"})
    if failures and not normalizers:
        raise AssetNormalizationError(
            "installed asset normalizers could not be loaded",
            operation="asset.normalize.discover",
            details={"failures": failures},
        )
    return tuple(normalizers)


def select_asset_normalizer(
    request: AssetNormalizationRequest,
    normalizers: Iterable[AssetNormalizer],
) -> AssetNormalizer:
    attempts: list[dict[str, object]] = []
    for normalizer in normalizers:
        try:
            accepted = normalizer.can_normalize(request)
        except Exception as exc:
            attempts.append({"normalizer_id": normalizer.normalizer_id, "error": f"{type(exc).__name__}: {exc}"})
            continue
        attempts.append({"normalizer_id": normalizer.normalizer_id, "accepted": accepted})
        if accepted:
            return normalizer
    raise AssetNormalizationError(
        "no installed asset normalizer accepts this request",
        operation="asset.normalize.select",
        backend_id=request.provider_id,
        details={"request": request.to_dict(), "attempts": attempts},
    )
