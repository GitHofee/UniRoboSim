import pytest

from unirobosim.api import CameraCalibrationSpec, CameraRenderExclusion, CameraSpec, EntityPath, ValidationError

K = (500.0, 0.0, 301.0, 0.0, 510.0, 227.0, 0.0, 0.0, 1.0)


def test_optional_camera_fields_keep_legacy_encoding_and_explicit_facts():
    legacy = CameraSpec()
    assert "calibration" not in legacy.to_dict() and "render_exclusions" not in legacy.to_dict()
    c = CameraCalibrationSpec("opencv_pinhole", K, (0.0,) * 8)
    x = CameraRenderExclusion(EntityPath("/robots/g2"), "wrist/visual/mesh")
    spec = CameraSpec(calibration=c, render_exclusions=(x,))
    assert spec.to_dict()["calibration"]["intrinsics"] == list(K)
    assert spec.to_dict()["render_exclusions"] == [
        {"entity_path": "/robots/g2", "relative_prim_path": "wrist/visual/mesh"}
    ]


@pytest.mark.parametrize(
    "k", [K[:8], (-1.0,) + K[1:], (500.0, 1.0) + K[2:], K[:-1] + (2.0,), (float("nan"),) + K[1:], (True,) + K[1:]]
)
def test_bad_intrinsics_fail(k):
    with pytest.raises(ValidationError):
        CameraCalibrationSpec("opencv_pinhole", k, (0.0,) * 8)


@pytest.mark.parametrize("path", ["", "/mesh", "../mesh", "a//b", "a/*", "a/**", "a/../b", "a.mesh", "a\\mesh"])
def test_exclusion_never_glob_or_escape(path):
    with pytest.raises(ValidationError):
        CameraRenderExclusion(EntityPath("/robots/g2"), path)


def test_distortion_and_duplicate_exclusion_validation():
    with pytest.raises(ValidationError):
        CameraCalibrationSpec("opencv_pinhole", K, (0.0,) * 5)
    x = CameraRenderExclusion(EntityPath("/robots/g2"), "mesh")
    with pytest.raises(ValidationError):
        CameraSpec(render_exclusions=(x, x))


def test_native_optics_are_explicit_positive_values_and_omitted_by_default():
    calibration = CameraCalibrationSpec('opencv_pinhole', K, (0.,) * 8, focal_length=2.4, focus_distance=1.)
    assert calibration.to_dict()['focal_length'] == 2.4
    assert 'horizontal_aperture' not in calibration.to_dict()
    with pytest.raises(ValidationError):
        CameraCalibrationSpec('opencv_pinhole', K, (0.,) * 8, fisheye_resolution_budget=0)
