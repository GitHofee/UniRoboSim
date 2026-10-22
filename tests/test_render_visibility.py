import pytest
from unirobosim import EntityPath, RenderVisibilityOverride, RenderVisibilityReceipt


@pytest.mark.parametrize(
    "path", ["", "/", "../ceilings", "a/../b", "/ceilings", "a.visibility", "a{v=x}", "a[0]", "a//b"]
)
def test_reject_path_escape(path):
    with pytest.raises(ValueError):
        RenderVisibilityOverride(EntityPath("/environment"), path)


def test_exact_supported_transition_and_receipt():
    request = RenderVisibilityOverride(EntityPath("/environment"), "ceilings")
    receipt = RenderVisibilityReceipt(
        request, "/World/env_0/environment/ceilings", "invisible", "invisible", "inherited", "inherited"
    )
    assert receipt.override == request
    with pytest.raises(ValueError):
        RenderVisibilityOverride(EntityPath("/environment"), "ceilings", visibility="invisible")
    with pytest.raises(ValueError):
        RenderVisibilityReceipt(request, "/x", "invisible", "invisible", "inherited", "invisible")
