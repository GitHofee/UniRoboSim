"""Backend-transparent logical asset manifests for EasyAPI."""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse

from unirobosim.api.errors import ValidationError
from unirobosim.api.frozen import FrozenMap

ASSET_BUNDLE_SCHEMA = "unirobosim.asset-bundle.v1"


def _invalid(message: str, **details: object) -> ValidationError:
    return ValidationError(message, operation="easy.asset_bundle", details=details)


def infer_media_type(uri: str) -> str:
    suffix = Path(urlparse(uri).path).suffix.lower()
    return {
        ".usd": "model/vnd.usd",
        ".usda": "model/vnd.usd",
        ".usdc": "model/vnd.usd",
        ".urdf": "model/vnd.urdf+xml",
        ".sdf": "model/vnd.sdf+xml",
        ".xml": "application/xml",
        ".mjcf": "model/vnd.mujoco.mjcf+xml",
        ".obj": "model/obj",
    }.get(suffix, "application/octet-stream")


def _normalize_variants(values: object, *, base_directory: Path | None = None) -> FrozenMap:
    if not isinstance(values, Mapping) or not values:
        raise _invalid("asset variants must be a non-empty mapping")
    normalized: dict[str, dict[str, str]] = {}
    for selector, raw in values.items():
        if not isinstance(selector, str) or not selector.strip():
            raise _invalid("asset variant selectors must be non-empty strings")
        if isinstance(raw, str):
            variant: Mapping[str, Any] = {"uri": raw}
        elif isinstance(raw, Mapping):
            variant = raw
        else:
            raise _invalid("asset variants must be URI strings or objects", selector=selector)
        uri = variant.get("uri")
        sha256 = variant.get("sha256")
        media_type = variant.get("media_type")
        if not isinstance(uri, str) or not uri.strip():
            raise _invalid("asset variant URI must be a non-empty string", selector=selector)
        if sha256 is not None and (
            not isinstance(sha256, str)
            or len(sha256) != 64
            or any(character not in "0123456789abcdefABCDEF" for character in sha256)
        ):
            raise _invalid("asset variant sha256 must be 64 hexadecimal characters", selector=selector)
        if media_type is not None and (not isinstance(media_type, str) or "/" not in media_type):
            raise _invalid("asset variant media_type must be a MIME type", selector=selector)
        parsed = urlparse(uri)
        if base_directory is not None and parsed.scheme == "" and not Path(uri).is_absolute():
            uri = str((base_directory / uri).resolve())
        normalized[selector] = {
            "uri": uri,
            "media_type": infer_media_type(uri) if media_type is None else media_type,
        }
        if sha256 is not None:
            normalized[selector]["sha256"] = sha256.lower()
    return FrozenMap(normalized)


@dataclass(frozen=True)
class ResolvedAsset:
    logical_name: str
    selector: str
    uri: str
    media_type: str
    sha256: str | None = None
    source_manifest: str | None = None
    conversion: FrozenMap | None = None
    normalization: FrozenMap | None = None

    def to_dict(self) -> dict[str, object]:
        result: dict[str, object] = {
            "logical_name": self.logical_name,
            "selector": self.selector,
            "uri": self.uri,
            "media_type": self.media_type,
        }
        if self.sha256 is not None:
            result["sha256"] = self.sha256
        if self.source_manifest is not None:
            result["source_manifest"] = self.source_manifest
        if self.conversion is not None:
            result["conversion"] = self.conversion.to_dict()
        if self.normalization is not None:
            result["normalization"] = self.normalization.to_dict()
        return result


@dataclass(frozen=True)
class AssetBundle:
    """One logical asset with backend/provider-specific native variants."""

    logical_name: str
    variants: FrozenMap | Mapping[str, object]
    schema_version: str = ASSET_BUNDLE_SCHEMA
    source_manifest: str | None = field(default=None, compare=False)

    def __post_init__(self) -> None:
        if not isinstance(self.logical_name, str) or not self.logical_name.strip():
            raise _invalid("logical asset name must be a non-empty string")
        if self.schema_version != ASSET_BUNDLE_SCHEMA:
            raise _invalid(
                "unsupported asset bundle schema",
                expected=ASSET_BUNDLE_SCHEMA,
                actual=self.schema_version,
            )
        if self.source_manifest is not None and not isinstance(self.source_manifest, str):
            raise _invalid("source_manifest must be a string path")
        object.__setattr__(self, "variants", _normalize_variants(self.variants))

    @classmethod
    def from_manifest(cls, path: str | Path) -> AssetBundle:
        manifest_path = Path(path).expanduser().resolve()
        try:
            raw = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise _invalid("asset manifest could not be read", path=str(manifest_path), error=str(exc)) from exc
        if not isinstance(raw, Mapping):
            raise _invalid("asset manifest root must be an object", path=str(manifest_path))
        schema = raw.get("schema_version")
        name = raw.get("name")
        if schema != ASSET_BUNDLE_SCHEMA:
            raise _invalid(
                "unsupported asset manifest schema",
                path=str(manifest_path),
                expected=ASSET_BUNDLE_SCHEMA,
                actual=schema,
            )
        if not isinstance(name, str):
            raise _invalid("asset manifest name must be a string", path=str(manifest_path))
        variants = _normalize_variants(raw.get("variants"), base_directory=manifest_path.parent)
        return cls(name, variants, schema_version=schema, source_manifest=str(manifest_path))

    def resolve(self, *, backend: str, provider_id: str) -> ResolvedAsset:
        candidates = tuple(
            dict.fromkeys(
                candidate
                for candidate in (
                    backend if backend != "auto" else None,
                    provider_id,
                    provider_id.rsplit(".", 1)[-1],
                )
                if candidate is not None
            )
        )
        selector = next((candidate for candidate in candidates if candidate in self.variants), None)
        if selector is None:
            raise _invalid(
                "logical asset has no variant for the selected backend",
                logical_name=self.logical_name,
                backend=backend,
                provider_id=provider_id,
                available_selectors=tuple(self.variants),
            )
        variant = self.variants[selector]
        assert isinstance(variant, FrozenMap)
        uri = variant["uri"]
        media_type = variant["media_type"]
        assert isinstance(uri, str)
        assert isinstance(media_type, str)
        expected_hash = variant.get("sha256")
        if expected_hash is not None:
            self._verify_local_hash(uri, str(expected_hash), selector)
        return ResolvedAsset(
            self.logical_name,
            selector,
            uri,
            media_type,
            None if expected_hash is None else str(expected_hash),
            self.source_manifest,
        )

    def source_for_conversion(self) -> ResolvedAsset:
        """Return the canonical/first USD variant when a native target is absent."""

        selectors = tuple(self.variants)
        preferred = tuple(selector for selector in ("source", "canonical", "usd", "isaaclab") if selector in selectors)
        candidates = (*preferred, *(selector for selector in selectors if selector not in preferred))
        for selector in candidates:
            variant = self.variants[selector]
            assert isinstance(variant, FrozenMap)
            uri = variant["uri"]
            media_type = variant["media_type"]
            assert isinstance(uri, str) and isinstance(media_type, str)
            if media_type != "model/vnd.usd":
                continue
            expected_hash = variant.get("sha256")
            if expected_hash is not None:
                self._verify_local_hash(uri, str(expected_hash), selector)
            return ResolvedAsset(
                self.logical_name,
                selector,
                uri,
                media_type,
                None if expected_hash is None else str(expected_hash),
                self.source_manifest,
            )
        raise _invalid(
            "logical asset has no USD source variant for conversion",
            logical_name=self.logical_name,
            available_selectors=selectors,
        )

    def _verify_local_hash(self, uri: str, expected: str, selector: str) -> None:
        parsed = urlparse(uri)
        if parsed.scheme not in {"", "file"}:
            return
        path = Path(unquote(parsed.path) if parsed.scheme == "file" else uri)
        try:
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
        except OSError as exc:
            raise _invalid(
                "selected local asset could not be hashed",
                logical_name=self.logical_name,
                selector=selector,
                uri=uri,
                error=str(exc),
            ) from exc
        actual = digest.hexdigest()
        if actual != expected:
            raise _invalid(
                "selected asset hash does not match its manifest",
                logical_name=self.logical_name,
                selector=selector,
                uri=uri,
                expected_sha256=expected,
                actual_sha256=actual,
            )

    def to_dict(self) -> dict[str, object]:
        assert isinstance(self.variants, FrozenMap)
        result: dict[str, object] = {
            "schema_version": self.schema_version,
            "name": self.logical_name,
            "variants": self.variants.to_dict(),
        }
        if self.source_manifest is not None:
            result["source_manifest"] = self.source_manifest
        return result
