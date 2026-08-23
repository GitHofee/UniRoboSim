from __future__ import annotations

import math

import pytest

from unirobosim import BuildLinkDynamics, BuildResourceManifest, ValidationError
from unirobosim.api._codec import canonical_json


class HostileStr(str):
    def __str__(self) -> str:
        raise KeyboardInterrupt("hostile string hook must not run")


class HostileInt(int):
    def __int__(self) -> int:
        raise KeyboardInterrupt("hostile integer hook must not run")


class HostileFloat(float):
    def __float__(self) -> float:
        raise KeyboardInterrupt("hostile float hook must not run")


class HostileTuple(tuple[object, ...]):
    def __iter__(self):  # type: ignore[no-untyped-def]
        raise KeyboardInterrupt("hostile tuple hook must not run")


class HostileList(list[object]):
    def __iter__(self):  # type: ignore[no-untyped-def]
        raise KeyboardInterrupt("hostile list hook must not run")


class HostileDict(dict[str, object]):
    def items(self):  # type: ignore[no-untyped-def]
        raise KeyboardInterrupt("hostile mapping hook must not run")


@pytest.mark.parametrize(
    "value",
    (
        HostileStr("text"),
        HostileInt(1),
        HostileFloat(1.0),
        HostileTuple((1,)),
        HostileList([1]),
        HostileDict({"key": "value"}),
        {1: "non-string-key"},
        float("nan"),
        float("inf"),
        "\ud800",
    ),
)
def test_canonical_codec_rejects_hostile_or_nonportable_values_without_hooks(value: object) -> None:
    with pytest.raises(ValidationError):
        canonical_json(value)


def test_canonical_codec_is_utf8_compact_sorted_and_normalizes_negative_zero() -> None:
    encoded = canonical_json({"β": (True, -0.0, None), "a": [1, 1.25]})
    assert encoded == '{"a":[1,1.25],"β":[true,0.0,null]}'
    assert encoded.encode("utf-8").decode("utf-8") == encoded
    assert not encoded.endswith("\n")


def test_canonical_codec_rejects_excessive_depth() -> None:
    value: object = None
    for _ in range(130):
        value = (value,)
    with pytest.raises(ValidationError, match="nesting budget"):
        canonical_json(value)


@pytest.mark.parametrize("value", ({"\ud800": 1}, 2**1024, 10**400))
def test_canonical_codec_rejects_non_utf8_keys_and_nonfinite_binary64_integers(value: object) -> None:
    with pytest.raises(ValidationError):
        canonical_json(value)


@pytest.mark.parametrize(
    "override",
    (
        {"mass_kg": True},
        {"mass_kg": HostileFloat(1.0)},
        {"center_of_mass_xyz_m": (0.0, 0.0, float("nan"))},
        {"principal_moments_kg_m2": (0.0, 1.0, 1.0)},
        {"normalization_recipe_sha256": "A" * 64},
    ),
)
def test_build_dynamics_rejects_hostile_exact_type_and_nonfinite_values(
    override: dict[str, object],
) -> None:
    values: dict[str, object] = {
        "link_id": "link.safe",
        "mass_kg": 1.0,
        "center_of_mass_xyz_m": (0.0, 0.0, 0.0),
        "inertia_tensor_com_link_kg_m2": (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        "principal_moments_kg_m2": (1.0, 1.0, 1.0),
        "principal_axes_xyzw": (0.0, 0.0, 0.0, 1.0),
        "normalization_recipe_sha256": "a" * 64,
    }
    values.update(override)
    with pytest.raises(ValidationError):
        BuildLinkDynamics(**values)  # type: ignore[arg-type]


def test_manifest_rejects_hostile_container_subclasses_without_iteration() -> None:
    with pytest.raises(ValidationError):
        BuildResourceManifest(HostileTuple(()))  # type: ignore[arg-type]


def test_manifest_schema_rejects_hostile_string_subclass_without_comparison() -> None:
    class HostileSchema(str):
        def __ne__(self, other: object) -> bool:
            raise KeyboardInterrupt("hostile schema comparison must not run")

    with pytest.raises(ValidationError):
        BuildResourceManifest(
            (),  # type: ignore[arg-type]
            schema_version=HostileSchema("unirobosim.build-resource-manifest/v1"),
        )


def test_inertia_acceptance_tolerance_is_finite_and_bounded() -> None:
    epsilon = math.ulp(1.0)
    dynamics = BuildLinkDynamics(
        "link.tolerance",
        1.0,
        (0.0, 0.0, 0.0),
        (1.0 + epsilon, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        (1.0, 1.0, 1.0),
        (0.0, 0.0, 0.0, 1.0),
        "b" * 64,
    )
    assert dynamics.inertia_tensor_com_link_kg_m2[0] == 1.0 + epsilon
