"""Recursively immutable JSON-compatible values."""

from __future__ import annotations

import math
from collections.abc import Iterable, Iterator, Mapping
from typing import Any, cast

from .errors import ValidationError


def _invalid(message: str, **details: object) -> ValidationError:
    return ValidationError(message, operation="frozen_json.validate", details=details)


def _actual_base(value: object, allowed: tuple[type, ...]) -> type | None:
    """Return an accepted builtin base without consulting instance hooks."""

    try:
        mro = type.mro(type(value))
    except BaseException:
        return None
    if type(mro) is not list or list.__len__(mro) > 256:
        return None
    for allowed_base in allowed:
        for index in range(list.__len__(mro)):
            if list.__getitem__(mro, index) is allowed_base:
                return allowed_base
    return None


def _exact_string(value: object, *, label: str) -> str:
    if _actual_base(value, (str,)) is not str:
        raise _invalid(f"{label} must be a string") from None
    try:
        result = str.__str__(cast(str, value))
    except BaseException:
        raise _invalid(f"{label} could not be detached") from None
    if type(result) is not str:
        raise _invalid(f"{label} could not be detached") from None
    return result


def _exact_integer(value: object) -> int:
    try:
        result = int.__int__(value)  # type: ignore[arg-type]
    except BaseException:
        raise _invalid("integer value could not be detached") from None
    if type(result) is not int:
        raise _invalid("integer value could not be detached") from None
    return result


def _exact_float(value: object) -> float:
    try:
        result = float.__float__(value)  # type: ignore[arg-type]
    except BaseException:
        raise _invalid("float value could not be detached") from None
    if type(result) is not float or not math.isfinite(result):
        raise _invalid("JSON float values must be finite") from None
    return result


def _sequence_items(value: object) -> tuple[object, ...] | None:
    base = _actual_base(value, (list, tuple))
    try:
        if base is list:
            source_list = cast(list[object], value)
            return tuple(list.__getitem__(source_list, index) for index in range(list.__len__(source_list)))
        if base is tuple:
            source_tuple = cast(tuple[object, ...], value)
            return tuple(tuple.__getitem__(source_tuple, index) for index in range(tuple.__len__(source_tuple)))
    except BaseException:
        raise _invalid("JSON array value could not be detached") from None
    return None


def freeze_json(value: Any) -> Any:
    """Return an immutable JSON-compatible representation.

    Dictionaries become :class:`FrozenMap`, arrays become tuples, and scalars are retained. Non-finite
    floats and non-JSON values are rejected so canonical fingerprints remain portable.
    """

    if value is None or type(value) is bool:
        return value
    base = _actual_base(value, (str, bool, int, float))
    if base is str:
        return _exact_string(value, label="JSON string value")
    if base is int:
        return _exact_integer(value)
    if base is float:
        return _exact_float(value)
    if type(value) is FrozenMap:
        return value
    if _actual_base(value, (dict, Mapping)) in (dict, Mapping):
        return FrozenMap(value)
    sequence = _sequence_items(value)
    if sequence is not None:
        return tuple(freeze_json(item) for item in sequence)
    raise _invalid("value is not JSON-compatible", value_type="unsupported")


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
            if values is None:
                raw_items: tuple[object, ...] = ()
            elif type(values) is FrozenMap:
                raw_items = object.__getattribute__(values, "_items")
            elif _actual_base(values, (dict,)) is dict:
                raw_items = tuple(dict.items(values))  # type: ignore[arg-type]
            elif _actual_base(values, (Mapping,)) is Mapping:
                raw_items = tuple(cast(Mapping[object, object], values).items())
            else:
                raw_items = tuple(values)
        except BaseException:
            raise _invalid("FrozenMap input must be a string-keyed mapping or pair iterable") from None

        source: dict[str, Any] = {}
        for raw_item in raw_items:
            pair = _sequence_items(raw_item)
            if pair is None:
                try:
                    pair = tuple(cast(Iterable[object], raw_item))
                except BaseException:
                    raise _invalid("FrozenMap entries must be key-value pairs") from None
            if len(pair) != 2:
                raise _invalid("FrozenMap entries must be key-value pairs") from None
            key = _exact_string(pair[0], label="FrozenMap key")
            source[key] = freeze_json(pair[1])
        self._items = tuple(sorted(source.items()))
        self._mapping = dict(self._items)

    def __getitem__(self, key: str) -> Any:
        return self._mapping[_exact_string(key, label="FrozenMap lookup key")]

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
