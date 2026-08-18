"""Versioned capability declarations and exact M0 negotiation."""

from __future__ import annotations

import hashlib
import json
import re
from collections.abc import Iterable, Iterator
from dataclasses import dataclass, field
from typing import Any

from .errors import ValidationError
from .frozen import FrozenMap, thaw_json

_CAPABILITY_ID = re.compile(r"^[a-z][a-z0-9]*(?:[._-][a-z0-9]+)*@[1-9][0-9]*$")


def _invalid(message: str, **details: object) -> ValidationError:
    return ValidationError(message, operation="capability.validate", details=details)


@dataclass(frozen=True, order=True)
class CapabilityId:
    value: str

    def __post_init__(self) -> None:
        if not isinstance(self.value, str) or not _CAPABILITY_ID.fullmatch(self.value):
            raise _invalid("invalid capability ID", value=self.value)

    @property
    def major(self) -> int:
        return int(self.value.rsplit("@", 1)[1])

    @property
    def name(self) -> str:
        return self.value.rsplit("@", 1)[0]

    def __str__(self) -> str:
        return self.value


@dataclass(frozen=True)
class CapabilityRequirement:
    capability: CapabilityId
    required: bool = True
    constraints: FrozenMap = field(default_factory=FrozenMap)
    reason: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.capability, CapabilityId):
            raise _invalid("requirement capability must be a CapabilityId")
        if not isinstance(self.required, bool):
            raise _invalid("requirement required flag must be boolean")
        if not isinstance(self.constraints, FrozenMap):
            raise _invalid("requirement constraints must be a FrozenMap")
        if self.reason is not None and (not isinstance(self.reason, str) or not self.reason.strip()):
            raise _invalid("requirement reason must be a non-empty string")

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.capability.value,
            "required": self.required,
            "constraints": self.constraints.to_dict(),
            "reason": self.reason,
        }


@dataclass(frozen=True)
class CapabilityDeclaration:
    capability: CapabilityId
    properties: FrozenMap = field(default_factory=FrozenMap)
    limitations: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.capability, CapabilityId):
            raise _invalid("declaration capability must be a CapabilityId")
        if not isinstance(self.properties, FrozenMap):
            raise _invalid("declaration properties must be a FrozenMap")
        try:
            limitations = tuple(self.limitations)
        except TypeError as exc:
            raise _invalid("capability limitations must be iterable") from exc
        if any(not isinstance(item, str) or not item.strip() for item in limitations):
            raise _invalid("capability limitations must be non-empty strings")
        object.__setattr__(self, "limitations", limitations)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.capability.value,
            "properties": self.properties.to_dict(),
            "limitations": list(self.limitations),
        }


@dataclass(frozen=True)
class CapabilityIssue:
    capability: CapabilityId
    required: bool
    reason: str
    expected: FrozenMap = field(default_factory=FrozenMap)
    actual: FrozenMap | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.capability, CapabilityId) or not isinstance(self.required, bool):
            raise _invalid("capability issue identity is invalid")
        if not isinstance(self.reason, str) or not self.reason:
            raise _invalid("capability issue reason must be a non-empty string")
        if not isinstance(self.expected, FrozenMap) or (
            self.actual is not None and not isinstance(self.actual, FrozenMap)
        ):
            raise _invalid("capability issue values must be FrozenMap instances")

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.capability.value,
            "required": self.required,
            "reason": self.reason,
            "expected": self.expected.to_dict(),
            "actual": None if self.actual is None else self.actual.to_dict(),
        }


@dataclass(frozen=True)
class NegotiationReport:
    matched: tuple[CapabilityId, ...]
    required_issues: tuple[CapabilityIssue, ...]
    optional_issues: tuple[CapabilityIssue, ...]

    def __post_init__(self) -> None:
        if any(not isinstance(item, CapabilityId) for item in self.matched):
            raise _invalid("negotiation matches must be CapabilityId values")
        if any(not isinstance(item, CapabilityIssue) for item in (*self.required_issues, *self.optional_issues)):
            raise _invalid("negotiation issues must be CapabilityIssue values")

    @property
    def accepted(self) -> bool:
        return not self.required_issues

    def to_dict(self) -> dict[str, Any]:
        return {
            "accepted": self.accepted,
            "matched": [item.value for item in self.matched],
            "required_issues": [item.to_dict() for item in self.required_issues],
            "optional_issues": [item.to_dict() for item in self.optional_issues],
        }


class CapabilitySet:
    """Immutable provider capability set."""

    __slots__ = ("_declarations", "_by_id")

    def __init__(self, declarations: Iterable[CapabilityDeclaration] = ()) -> None:
        try:
            raw_items = tuple(declarations)
        except TypeError as exc:
            raise _invalid("capability declarations must be iterable") from exc
        if any(not isinstance(item, CapabilityDeclaration) for item in raw_items):
            raise _invalid("capability set contains a non-declaration")
        items = tuple(sorted(raw_items, key=lambda item: item.capability.value))
        ids = tuple(item.capability for item in items)
        if len(ids) != len(set(ids)):
            raise _invalid("capability declarations must be unique", ids=[item.value for item in ids])
        self._declarations = items
        self._by_id = {item.capability: item for item in items}

    def __iter__(self) -> Iterator[CapabilityDeclaration]:
        return iter(self._declarations)

    def __len__(self) -> int:
        return len(self._declarations)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, CapabilitySet):
            return NotImplemented
        return self._declarations == other._declarations

    def __hash__(self) -> int:
        return hash(self._declarations)

    def __repr__(self) -> str:
        return f"CapabilitySet({self._declarations!r})"

    def get(self, capability: CapabilityId) -> CapabilityDeclaration | None:
        return self._by_id.get(capability)

    @property
    def digest(self) -> str:
        payload = [item.to_dict() for item in self._declarations]
        canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()

    def negotiate(self, requirements: Iterable[CapabilityRequirement]) -> NegotiationReport:
        try:
            requested = tuple(requirements)
        except TypeError as exc:
            raise _invalid("capability requirements must be iterable") from exc
        if any(not isinstance(item, CapabilityRequirement) for item in requested):
            raise _invalid("negotiation contains a non-requirement")
        ids = tuple(item.capability for item in requested)
        if len(ids) != len(set(ids)):
            raise _invalid("capability requirements must be unique", ids=[item.value for item in ids])

        matched: list[CapabilityId] = []
        required_issues: list[CapabilityIssue] = []
        optional_issues: list[CapabilityIssue] = []
        for requirement in requested:
            declaration = self._by_id.get(requirement.capability)
            issue: CapabilityIssue | None = None
            if declaration is None:
                issue = CapabilityIssue(
                    capability=requirement.capability,
                    required=requirement.required,
                    reason="missing",
                    expected=requirement.constraints,
                )
            else:
                mismatches: dict[str, Any] = {}
                for key, expected in requirement.constraints.items():
                    if key not in declaration.properties or declaration.properties[key] != expected:
                        mismatches[key] = {
                            "expected": thaw_json(expected),
                            "actual": thaw_json(declaration.properties.get(key)),
                        }
                if mismatches:
                    issue = CapabilityIssue(
                        capability=requirement.capability,
                        required=requirement.required,
                        reason="property_mismatch",
                        expected=FrozenMap(mismatches),
                        actual=declaration.properties,
                    )
            if issue is None:
                matched.append(requirement.capability)
            elif requirement.required:
                required_issues.append(issue)
            else:
                optional_issues.append(issue)

        return NegotiationReport(tuple(matched), tuple(required_issues), tuple(optional_issues))
