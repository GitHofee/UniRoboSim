"""Scalar validation retains portable, detached values on its common fast path."""

import math

import pytest

from unirobosim import PlanningPose, PlanningSceneContractError
from unirobosim.api import planning_scene as contract


@pytest.mark.parametrize('scalar', [True, False, float('nan'), float('inf'), -float('inf'), 10**1000])
def test_pose_rejects_non_portable_numeric_scalars(scalar):
    with pytest.raises(PlanningSceneContractError):
        PlanningPose('frame.world', (scalar, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def test_scalar_subclass_hooks_are_not_executed_and_values_are_detached():
    class HostileString(str):
        def __str__(self):
            raise AssertionError('override called')

        def isascii(self):
            raise AssertionError('override called')

        @property
        def __class__(self):
            raise AssertionError('override called')

    class HostileFloat(float):
        def __float__(self):
            raise AssertionError('override called')

        @property
        def __class__(self):
            raise AssertionError('override called')

    class HostileInt(int):
        def __int__(self):
            raise AssertionError('override called')

        @property
        def __class__(self):
            raise AssertionError('override called')

    pose = PlanningPose(
        HostileString('frame.world'),
        (HostileFloat(1.25), HostileInt(2), -0.0),
        (0.0, 0.0, 0.0, HostileFloat(-1.0)),
    )
    assert type(pose.frame_id) is str
    assert pose.position_m == (1.25, 2.0, 0.0)
    assert all(type(value) is float for value in pose.position_m)
    assert math.copysign(1.0, pose.position_m[2]) == 1.0
    assert pose.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)


@pytest.mark.parametrize('value', ['\ud800', 'prefix\udfff', 'x\x00', 'x' * 513])
def test_text_fast_path_preserves_unicode_and_length_rejection(value):
    with pytest.raises(PlanningSceneContractError):
        contract._text(value, 'name')


def test_text_keeps_valid_non_ascii_and_rejects_it_in_identifiers():
    assert contract._text('机器人', 'name') == '机器人'
    with pytest.raises(PlanningSceneContractError):
        PlanningPose('frame.机器人', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


@pytest.mark.parametrize('value', ['text', 7, 1.5, True])
def test_classifier_retains_allowed_base_precedence(value):
    assert contract._actual_base(value, (object, type(value))) is object
    assert contract._actual_base(value, (type(value), object)) is type(value)
    assert contract._actual_base(True, (int, bool)) is int
    assert contract._actual_base(True, (bool, int)) is bool


def _reference_actual_base(value, allowed):
    """Unoptimized 0.10.7 classifier: differential regression oracle."""
    try:
        mro = type.mro(type(value))
    except BaseException:
        return contract._TYPE_INSPECTION_FAILED
    if type(mro) is not list or list.__len__(mro) > 256:
        return contract._TYPE_INSPECTION_FAILED
    for allowed_base in allowed:
        for index in range(list.__len__(mro)):
            if list.__getitem__(mro, index) is allowed_base:
                return allowed_base
    return None


def test_classifier_matches_original_for_subclasses_hostile_meta_and_allowed_order():
    from itertools import permutations

    class HostileMeta(type):
        def __eq__(self, other):
            raise AssertionError('type equality hook called')

        def __hash__(self):
            raise AssertionError('type hash hook called')

    class Integer(int, metaclass=HostileMeta):
        @property
        def __class__(self):
            raise AssertionError('class property called')

    class Float(float):
        pass

    class Text(str):
        pass

    class Spoof:
        @property
        def __class__(self):
            return str

    values = ('', 'robot', 0, -1, 2**128, 0.0, -0.0, math.nan, math.inf,
              True, False, Integer(3), Float(1.5), Text('robot'), Spoof(), None, ())
    bases = (bool, int, float, str, object, Integer)
    for count in range(4):
        for allowed in permutations(bases, count):
            for value in values:
                assert contract._actual_base(value, allowed) is _reference_actual_base(value, allowed)


def test_classifier_bypasses_metaclass_mro_override_and_retains_depth_budget():
    class CustomMro(type):
        forbid = False

        def mro(cls):
            if CustomMro.forbid:
                raise AssertionError('metaclass override called')
            return super().mro()

    class Integer(int, metaclass=CustomMro):
        pass

    CustomMro.forbid = True
    assert contract._actual_base(Integer(1), (bool, int, float)) is int
    deep = int
    for index in range(256):
        deep = type(f'Depth{index}', (deep,), {})
    assert contract._actual_base(deep(1), (int,)) is contract._TYPE_INSPECTION_FAILED


@pytest.mark.parametrize('value', ['', '\x00', 'a\x00b', 'a' * 512, 'a' * 513,
                                  '机器人', '\U0001f916' * 512, '\ud800', 'x\udfff'])
@pytest.mark.parametrize('identifier,opaque', [(False, False), (True, False), (False, True)])
def test_text_result_matches_original_surrogate_scan(value, identifier, opaque):
    # Compare against the pre-optimization Unicode scan and classifier.

    def result(function):
        try:
            return ('ok', function(value, 'text', identifier=identifier, opaque=opaque))
        except PlanningSceneContractError as error:
            return ('error', str(error))

    actual = result(contract._text)
    expected = result(_reference_text)
    assert actual == expected
    if '\ud800' in value or '\udfff' in value:
        assert actual[0] == 'error'


def test_text_preserves_utf8_budget_after_ascii_shortcut(monkeypatch):
    monkeypatch.setattr(contract, '_MAX_TEXT_BYTES', 5)
    assert contract._text('abcde', 'text') == 'abcde'
    with pytest.raises(PlanningSceneContractError, match='UTF-8 byte budget'):
        contract._text('机器人', 'text')


def _reference_text(value: object, label: str, *, identifier: bool = False, opaque: bool = False) -> str:
    if _reference_actual_base(value, (str,)) is not str:
        raise contract._invalid(f"{label} must be a string") from None
    byte_limit = contract._MAX_ID_BYTES if identifier or opaque else contract._MAX_TEXT_BYTES
    try:
        source_length = str.__len__(value)  # type: ignore[arg-type]
    except BaseException:
        source_length = contract._MAX_TEXT_CODEPOINTS + 1
    if source_length > min(contract._MAX_TEXT_CODEPOINTS, byte_limit):
        raise contract._invalid(f"{label} exceeds its text budget") from None
    try:
        canonical = str.__str__(value)
    except BaseException:
        canonical = None
    if (
        type(canonical) is not str
        or not canonical
        or len(canonical) > contract._MAX_TEXT_CODEPOINTS
        or "\x00" in canonical
    ):
        raise contract._invalid(f"{label} must be bounded non-empty portable text") from None
    if any(0xD800 <= ord(character) <= 0xDFFF for character in canonical):
        raise contract._invalid(f"{label} must contain valid Unicode") from None
    encoded = canonical.encode("utf-8")
    if len(encoded) > byte_limit:
        raise contract._invalid(f"{label} exceeds its UTF-8 byte budget") from None
    if identifier and contract._IDENTIFIER.fullmatch(canonical) is None:
        raise contract._invalid(f"{label} must be a stable ASCII identifier") from None
    if opaque and contract._OPAQUE_IDENTIFIER.fullmatch(canonical) is None:
        raise contract._invalid(f"{label} must be an opaque scoped identifier") from None
    return canonical
