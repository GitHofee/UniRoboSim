"""Recursively immutable JSON-compatible values."""

from __future__ import annotations

import math
from collections.abc import Iterable, Iterator, Mapping
from typing import Any, cast

from .errors import ValidationError

_MAX_CONTAINER_ITEMS = 100_000
_FAILED_INPUT = object()
_FAILED_BUDGET = object()
_FAILED_UNSUPPORTED = object()
_FAILED_NONFINITE = object()


def _invalid(message: str, **details: object) -> ValidationError:
    return ValidationError(message, operation="frozen_json.validate", details=details)


def _scrub_failure(error: BaseException) -> None:
    """Drop every standard edge from a replaced caller-controlled failure."""

    try:
        state = BaseException.__getattribute__(error, "__dict__")
        if type(state) is dict:
            dict.clear(state)
    except BaseException:
        pass
    for name, value in (
        ("args", ()),
        ("__traceback__", None),
        ("__cause__", None),
        ("__context__", None),
    ):
        try:
            BaseException.__setattr__(error, name, value)
        except BaseException:
            pass


def _actual_base(value: object, allowed: tuple[type, ...]) -> type | None:
    """Return an accepted builtin base without consulting instance hooks."""

    try:
        mro = type.mro(type(value))
    except BaseException as caught:
        _scrub_failure(caught)
        return None
    if type(mro) is not list or list.__len__(mro) > 256:
        return None
    for allowed_base in allowed:
        for index in range(list.__len__(mro)):
            if list.__getitem__(mro, index) is allowed_base:
                return allowed_base
    return None


class _FreezeBudget:
    __slots__ = ("remaining",)

    def __init__(self) -> None:
        self.remaining = _MAX_CONTAINER_ITEMS

    def consume(self, count: int = 1) -> bool:
        if type(count) is not int or count < 0 or count > self.remaining:
            return False
        self.remaining -= count
        return True


def _detached_string(value: object) -> str | object:
    if _actual_base(value, (str,)) is not str:
        return _FAILED_UNSUPPORTED
    try:
        result = str.__str__(cast(str, value))
    except BaseException as caught:
        _scrub_failure(caught)
        return _FAILED_INPUT
    if type(result) is not str:
        return _FAILED_INPUT
    return result


def _detached_integer(value: object) -> int | object:
    try:
        result = int.__int__(value)  # type: ignore[arg-type]
    except BaseException as caught:
        _scrub_failure(caught)
        return _FAILED_INPUT
    if type(result) is not int:
        return _FAILED_INPUT
    return result


def _detached_float(value: object) -> float | object:
    try:
        result = float.__float__(value)  # type: ignore[arg-type]
    except BaseException as caught:
        _scrub_failure(caught)
        return _FAILED_INPUT
    if type(result) is not float:
        return _FAILED_INPUT
    if not math.isfinite(result):
        return _FAILED_NONFINITE
    return result


def _pair_items(value: object) -> tuple[object, object] | object:
    """Detach one pair shape while reading at most three arbitrary items."""

    base = _actual_base(value, (list, tuple, str))
    try:
        if base is list:
            source_list = cast(list[object], value)
            if list.__len__(source_list) != 2:
                return _FAILED_INPUT
            return list.__getitem__(source_list, 0), list.__getitem__(source_list, 1)
        if base is tuple:
            source_tuple = cast(tuple[object, ...], value)
            if tuple.__len__(source_tuple) != 2:
                return _FAILED_INPUT
            return tuple.__getitem__(source_tuple, 0), tuple.__getitem__(source_tuple, 1)
        if base is str:
            source_text = cast(str, value)
            if str.__len__(source_text) != 2:
                return _FAILED_INPUT
            return str.__getitem__(source_text, 0), str.__getitem__(source_text, 1)
    except BaseException as caught:
        _scrub_failure(caught)
        return _FAILED_INPUT

    iterator: Iterator[object] | None = None
    detached: list[object] = []
    try:
        iterator = iter(cast(Iterable[object], value))
        for _ in range(3):
            try:
                detached.append(next(iterator))
            except StopIteration as stopped:
                _scrub_failure(stopped)
                break
    except BaseException as caught:
        _scrub_failure(caught)
        iterator = None
        list.clear(detached)
        return _FAILED_INPUT
    iterator = None
    if list.__len__(detached) != 2:
        list.clear(detached)
        return _FAILED_INPUT
    result = (list.__getitem__(detached, 0), list.__getitem__(detached, 1))
    list.clear(detached)
    return result


def _freeze_sequence(value: object, budget: _FreezeBudget) -> tuple[Any, ...] | object:
    base = _actual_base(value, (list, tuple))
    source_list: list[object] | None = None
    source_tuple: tuple[object, ...] | None = None
    try:
        if base is list:
            source_list = cast(list[object], value)
            length = list.__len__(source_list)
        elif base is tuple:
            source_tuple = cast(tuple[object, ...], value)
            length = tuple.__len__(source_tuple)
        else:
            return _FAILED_UNSUPPORTED
    except BaseException as caught:
        _scrub_failure(caught)
        return _FAILED_INPUT
    if not budget.consume(length):
        return _FAILED_BUDGET
    result: list[Any] = []
    for index in range(length):
        try:
            item = (
                list.__getitem__(source_list, index)
                if source_list is not None
                else tuple.__getitem__(cast(tuple[object, ...], source_tuple), index)
            )
        except BaseException as caught:
            _scrub_failure(caught)
            list.clear(result)
            return _FAILED_INPUT
        frozen = _freeze_value(item, budget)
        item = None
        if (
            frozen is _FAILED_INPUT
            or frozen is _FAILED_BUDGET
            or frozen is _FAILED_UNSUPPORTED
            or frozen is _FAILED_NONFINITE
        ):
            list.clear(result)
            return frozen
        result.append(frozen)
    return tuple(result)


def _freeze_mapping_items(
    values: object,
    budget: _FreezeBudget,
    *,
    allow_none: bool,
) -> tuple[tuple[str, Any], ...] | object:
    if values is None:
        return () if allow_none else _FAILED_UNSUPPORTED
    iterator: Iterator[object] | None = None
    iterable: object | None = None
    try:
        if type(values) is FrozenMap:
            iterable = object.__getattribute__(values, "_items")
        else:
            base = _actual_base(values, (dict, Mapping))
            if base is dict:
                iterable = dict.items(cast(dict[object, object], values))
            elif base is Mapping:
                iterable = cast(Mapping[object, object], values).items()
            else:
                iterable = values
        iterator = iter(cast(Iterable[object], iterable))
    except BaseException as caught:
        _scrub_failure(caught)
        iterable = None
        iterator = None
        return _FAILED_INPUT

    source: dict[str, Any] = {}
    while True:
        try:
            raw_item = next(iterator)
        except StopIteration as stopped:
            _scrub_failure(stopped)
            break
        except BaseException as caught:
            _scrub_failure(caught)
            iterator = None
            iterable = None
            dict.clear(source)
            return _FAILED_INPUT
        if not budget.consume():
            raw_item = None
            iterator = None
            iterable = None
            dict.clear(source)
            return _FAILED_BUDGET
        pair = _pair_items(raw_item)
        raw_item = None
        if pair is _FAILED_INPUT or pair is _FAILED_UNSUPPORTED:
            iterator = None
            iterable = None
            dict.clear(source)
            return _FAILED_INPUT
        assert type(pair) is tuple
        raw_key = tuple.__getitem__(pair, 0)
        raw_value = tuple.__getitem__(pair, 1)
        key = _detached_string(raw_key)
        raw_key = None
        if key is _FAILED_INPUT or key is _FAILED_UNSUPPORTED:
            raw_value = None
            pair = None
            iterator = None
            iterable = None
            dict.clear(source)
            return _FAILED_INPUT
        assert type(key) is str
        frozen = _freeze_value(raw_value, budget)
        raw_value = None
        pair = None
        if (
            frozen is _FAILED_INPUT
            or frozen is _FAILED_BUDGET
            or frozen is _FAILED_UNSUPPORTED
            or frozen is _FAILED_NONFINITE
        ):
            iterator = None
            iterable = None
            dict.clear(source)
            return frozen
        source[key] = frozen
    iterator = None
    iterable = None
    return tuple(sorted(dict.items(source)))


def _frozen_map_from_items(items: tuple[tuple[str, Any], ...]) -> FrozenMap:
    result = object.__new__(FrozenMap)
    object.__setattr__(result, "_items", items)
    object.__setattr__(result, "_mapping", dict(items))
    return result


def _freeze_value(value: object, budget: _FreezeBudget) -> Any:
    if value is None or type(value) is bool:
        return value
    base = _actual_base(value, (str, bool, int, float))
    if base is str:
        return _detached_string(value)
    if base is int:
        return _detached_integer(value)
    if base is float:
        return _detached_float(value)
    mapping_base = _actual_base(value, (dict, Mapping))
    if type(value) is FrozenMap or mapping_base is dict or mapping_base is Mapping:
        items = _freeze_mapping_items(value, budget, allow_none=False)
        if type(items) is tuple:
            return _frozen_map_from_items(cast(tuple[tuple[str, Any], ...], items))
        return items
    return _freeze_sequence(value, budget)


def _freeze_value_boundary(value: object) -> Any:
    try:
        return _freeze_value(value, _FreezeBudget())
    except BaseException as caught:
        _scrub_failure(caught)
        return _FAILED_INPUT


def _freeze_mapping_boundary(values: object) -> tuple[tuple[str, Any], ...] | object:
    try:
        return _freeze_mapping_items(values, _FreezeBudget(), allow_none=True)
    except BaseException as caught:
        _scrub_failure(caught)
        return _FAILED_INPUT


def _raise_freeze_failure(failure: object, *, mapping: bool) -> None:
    if failure is _FAILED_BUDGET:
        raise _invalid("JSON container item budget exceeded") from None
    if failure is _FAILED_NONFINITE:
        raise _invalid("JSON float values must be finite") from None
    if mapping:
        raise _invalid("FrozenMap input must contain JSON-compatible string-keyed pairs") from None
    if failure is _FAILED_UNSUPPORTED:
        raise _invalid("value is not JSON-compatible", value_type="unsupported") from None
    raise _invalid("JSON value could not be detached") from None


def freeze_json(value: Any) -> Any:
    """Return an immutable JSON-compatible representation.

    Dictionaries become :class:`FrozenMap`, arrays become tuples, and scalars are detached to exact
    builtins.  One construction may consume at most 100,000 container occurrences across the whole
    value graph; duplicate mapping entries count individually.
    """

    result = _freeze_value_boundary(value)
    value = None
    if (
        result is _FAILED_INPUT
        or result is _FAILED_BUDGET
        or result is _FAILED_UNSUPPORTED
        or result is _FAILED_NONFINITE
    ):
        _raise_freeze_failure(result, mapping=False)
    return result


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
        result = _freeze_mapping_boundary(values)
        values = None
        if type(result) is not tuple:
            _raise_freeze_failure(result, mapping=True)
        items = cast(tuple[tuple[str, Any], ...], result)
        self._items = items
        self._mapping = dict(items)

    def __getitem__(self, key: str) -> Any:
        canonical = _detached_string(key)
        key = ""
        if type(canonical) is not str:
            _raise_freeze_failure(canonical, mapping=True)
        return self._mapping[cast(str, canonical)]

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
