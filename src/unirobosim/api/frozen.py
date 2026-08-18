"""Recursively immutable JSON-compatible values."""

from __future__ import annotations

import math
from collections.abc import Iterable, Iterator, Mapping
from typing import Any, cast

from .errors import ValidationError


def _invalid(message: str, **details: object) -> ValidationError:
    return ValidationError(message, operation="frozen_json.validate", details=details)


def freeze_json(value: Any) -> Any:
    """Return an immutable JSON-compatible representation.

    Dictionaries become :class:`FrozenMap`, arrays become tuples, and scalars are retained. Non-finite
    floats and non-JSON values are rejected so canonical fingerprints remain portable.
    """

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise _invalid("JSON float values must be finite")
        return value
    if isinstance(value, FrozenMap):
        return value
    if isinstance(value, Mapping):
        return FrozenMap(value)
    if isinstance(value, (list, tuple)):
        return tuple(freeze_json(item) for item in value)
    raise _invalid("value is not JSON-compatible", value_type=type(value).__name__)


def thaw_json(value: Any) -> Any:
    """Return mutable JSON-compatible containers for serialization."""

    if isinstance(value, FrozenMap):
        return {key: thaw_json(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [thaw_json(item) for item in value]
    return value


class FrozenMap(Mapping[str, Any]):
    """A deterministic, hashable mapping with recursively frozen values."""

    __slots__ = ("_items", "_mapping")

    def __init__(self, values: Mapping[str, Any] | Iterable[tuple[str, Any]] | None = None) -> None:
        try:
            source = {} if values is None else dict(values)
        except (TypeError, ValueError) as exc:
            raise _invalid("FrozenMap input must be a string-keyed mapping or pair iterable") from exc
        for key in source:
            if not isinstance(key, str):
                raise _invalid("FrozenMap keys must be strings", key_type=type(key).__name__)
        self._items = tuple(sorted((key, freeze_json(value)) for key, value in source.items()))
        self._mapping = dict(self._items)

    def __getitem__(self, key: str) -> Any:
        return self._mapping[key]

    def __iter__(self) -> Iterator[str]:
        return iter(self._mapping)

    def __len__(self) -> int:
        return len(self._items)

    def __hash__(self) -> int:
        return hash(self._items)

    def __repr__(self) -> str:
        content = ", ".join(f"{key!r}: {value!r}" for key, value in self._items)
        return f"FrozenMap({{{content}}})"

    def to_dict(self) -> dict[str, Any]:
        return cast(dict[str, Any], thaw_json(self))
