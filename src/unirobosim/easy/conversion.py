"""Optional, provider-aware asset conversion contracts for EasyAPI."""

from __future__ import annotations

import os
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from enum import StrEnum
from importlib import metadata
from pathlib import Path
from typing import Protocol, runtime_checkable

from unirobosim.api.errors import AssetConversionError, ValidationError
from unirobosim.api.frozen import FrozenMap
from unirobosim.api.values import EntityKind


class AssetPolicy(StrEnum):
    """Behavior when an entity asset is not native to the selected provider."""

    PREBUILT_ONLY = "prebuilt_only"
    CONVERT_IF_NEEDED = "convert_if_needed"


@dataclass(frozen=True)
class AssetConversionRequest:
    source_uri: str
    source_media_type: str
    target_backend: str
    provider_id: str
    entity_kind: EntityKind
    cache_directory: str
    options: FrozenMap | Mapping[str, object] = field(default_factory=FrozenMap)

    def __post_init__(self) -> None:
        if not isinstance(self.source_uri, str) or not self.source_uri.strip():
            raise ValidationError("conversion source URI must be non-empty", operation="asset.convert.validate")
        if not isinstance(self.source_media_type, str) or "/" not in self.source_media_type:
            raise ValidationError("conversion source media type is invalid", operation="asset.convert.validate")
        if not isinstance(self.target_backend, str) or not self.target_backend.strip():
            raise ValidationError("conversion target backend must be non-empty", operation="asset.convert.validate")
        if not isinstance(self.provider_id, str) or not self.provider_id.strip():
            raise ValidationError("conversion provider ID must be non-empty", operation="asset.convert.validate")
        if not isinstance(self.entity_kind, EntityKind):
            raise ValidationError("conversion entity kind is invalid", operation="asset.convert.validate")
        if not isinstance(self.cache_directory, str) or not self.cache_directory.strip():
            raise ValidationError("conversion cache directory must be non-empty", operation="asset.convert.validate")
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
            "cache_directory": self.cache_directory,
            "options": FrozenMap(self.options).to_dict(),
        }


@dataclass(frozen=True)
class AssetConversionResult:
    uri: str
    media_type: str
    converter_id: str
    converter_version: str
    source_sha256: str
    recipe_sha256: str
    output_sha256: str
    report_uri: str
    cache_hit: bool = False
    warnings: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        text_fields = (
            self.uri,
            self.media_type,
            self.converter_id,
            self.converter_version,
            self.source_sha256,
            self.recipe_sha256,
            self.output_sha256,
            self.report_uri,
        )
        if any(not isinstance(value, str) or not value.strip() for value in text_fields):
            raise ValidationError(
                "conversion result fields must be non-empty strings",
                operation="asset.convert.result",
            )
        if "/" not in self.media_type:
            raise ValidationError("conversion result media type is invalid", operation="asset.convert.result")
        for name, digest in (
            ("source_sha256", self.source_sha256),
            ("recipe_sha256", self.recipe_sha256),
            ("output_sha256", self.output_sha256),
        ):
            if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest.lower()):
                raise ValidationError(
                    f"conversion result {name} must be a SHA-256 digest", operation="asset.convert.result"
                )
        if not isinstance(self.cache_hit, bool) or any(
            not isinstance(warning, str) or not warning for warning in self.warnings
        ):
            raise ValidationError("conversion result cache/warnings are invalid", operation="asset.convert.result")

    def to_dict(self) -> dict[str, object]:
        return {
            "uri": self.uri,
            "media_type": self.media_type,
            "converter_id": self.converter_id,
            "converter_version": self.converter_version,
            "source_sha256": self.source_sha256,
            "recipe_sha256": self.recipe_sha256,
            "output_sha256": self.output_sha256,
            "report_uri": self.report_uri,
            "cache_hit": self.cache_hit,
            "warnings": list(self.warnings),
        }


@runtime_checkable
class AssetConverter(Protocol):
    @property
    def converter_id(self) -> str: ...

    def can_convert(self, request: AssetConversionRequest) -> bool: ...

    def convert(self, request: AssetConversionRequest) -> AssetConversionResult: ...


def default_asset_cache_directory() -> str:
    configured = os.environ.get("UNIROBOSIM_ASSET_CACHE")
    return str(Path(configured).expanduser()) if configured else str(Path.home() / ".cache" / "unirobosim" / "assets")


def discover_asset_converters() -> tuple[AssetConverter, ...]:
    converters: list[AssetConverter] = []
    failures: list[dict[str, str]] = []
    points = tuple(sorted(metadata.entry_points(group="unirobosim.asset_converters"), key=lambda item: item.name))
    for point in points:
        try:
            converter = point.load()()
            if not isinstance(converter, AssetConverter):
                raise TypeError("entry point did not return an AssetConverter")
            converters.append(converter)
        except Exception as exc:
            failures.append({"entry_point": point.name, "error": f"{type(exc).__name__}: {exc}"})
    if failures and not converters:
        raise AssetConversionError(
            "installed asset converters could not be loaded",
            operation="asset.convert.discover",
            details={"failures": failures},
        )
    return tuple(converters)


def select_asset_converter(
    request: AssetConversionRequest,
    converters: Iterable[AssetConverter],
) -> AssetConverter:
    attempts: list[dict[str, object]] = []
    for converter in converters:
        try:
            accepted = converter.can_convert(request)
        except Exception as exc:
            attempts.append({"converter_id": converter.converter_id, "error": f"{type(exc).__name__}: {exc}"})
            continue
        attempts.append({"converter_id": converter.converter_id, "accepted": accepted})
        if accepted:
            return converter
    raise AssetConversionError(
        "no installed asset converter accepts this request",
        operation="asset.convert.select",
        backend_id=request.provider_id,
        details={"request": request.to_dict(), "attempts": attempts},
    )
