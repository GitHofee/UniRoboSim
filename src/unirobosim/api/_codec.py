"""Hostile-safe canonical JSON used by portable Core values."""

from __future__ import annotations

import json
import math
from typing import TypeAlias

from .errors import ValidationError

CanonicalScalar: TypeAlias = None | bool | int | float | str
CanonicalValue: TypeAlias = CanonicalScalar | list["CanonicalValue"] | dict[str, "CanonicalValue"]

_MAX_DEPTH = 128
_MAX_ITEMS = 1_000_000


def _invalid(message: str) -> ValidationError:
    return ValidationError(message, operation="canonical_json.encode")


def _copy_exact(value: object, *, depth: int, budget: list[int]) -> CanonicalValue:
    if depth > _MAX_DEPTH:
        raise _invalid("canonical value exceeds its nesting budget") from None
    budget[0] += 1
    if budget[0] > _MAX_ITEMS:
        raise _invalid("canonical value exceeds its item budget") from None
    if value is None or type(value) is bool:
        return value
    if type(value) is int:
        try:
            finite = float(value)
        except OverflowError:
            raise _invalid("canonical integers must fit finite binary64") from None
        if not math.isfinite(finite):
            raise _invalid("canonical integers must fit finite binary64") from None
        return value
    if type(value) is float:
        if not math.isfinite(value):
            raise _invalid("canonical floating values must be finite") from None
        return 0.0 if value == 0.0 else value
    if type(value) is str:
        if any(0xD800 <= ord(character) <= 0xDFFF for character in value):
            raise _invalid("canonical strings must contain valid Unicode") from None
        return value
    if type(value) is list or type(value) is tuple:
        return [_copy_exact(item, depth=depth + 1, budget=budget) for item in value]
    if type(value) is dict:
        result: dict[str, CanonicalValue] = {}
        for key, item in value.items():
            if type(key) is not str:
                raise _invalid("canonical object keys must be exact strings") from None
            if any(0xD800 <= ord(character) <= 0xDFFF for character in key):
                raise _invalid("canonical object keys must contain valid Unicode") from None
            result[key] = _copy_exact(item, depth=depth + 1, budget=budget)
        return result
    raise _invalid("canonical values must use exact built-in JSON types") from None


def canonical_json(value: object) -> str:
    """Return canonical UTF-8 JSON text without a BOM or trailing newline."""

    portable = _copy_exact(value, depth=0, budget=[0])
    return json.dumps(
        portable,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )
