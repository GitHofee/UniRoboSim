from dataclasses import replace

import pytest

from unirobosim import ConnectionProperties, ValidationError


@pytest.mark.parametrize("field", tuple(ConnectionProperties.__dataclass_fields__))
@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf"), True, "1", None, 10**1000])
def test_connection_numeric_contract(field, value):
    properties = ConnectionProperties(300.0, 1.0, 1.0, 0.002, 3.0, 0.01, 0.008, 0.35, 0.0015, 0.02)
    with pytest.raises(ValidationError):
        replace(properties, **{field: value})
