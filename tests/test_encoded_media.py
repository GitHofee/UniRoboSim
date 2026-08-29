from __future__ import annotations

import pytest

from unirobosim import (
    EncodedSensorFrame,
    EncodedSensorRequest,
    EncodedSensorWorld,
    EntityHandle,
    EntityKind,
    EntityPath,
    Tick,
)


def _camera() -> EntityHandle:
    return EntityHandle(
        "provider",
        "session",
        "world",
        1,
        EntityPath("/camera"),
        EntityKind.CAMERA_SENSOR,
        "native-camera",
    )


def test_jpeg_request_and_frame_are_strict_and_immutable() -> None:
    request = EncodedSensorRequest(_camera(), quality=91)
    frame = EncodedSensorFrame(
        handle=request.handle,
        payload=b"\xff\xd8\xffpayload",
        width_px=1280,
        height_px=720,
        encoding=request.encoding,
        quality=request.quality,
        color_space=request.color_space,
        chroma_subsampling=request.chroma_subsampling,
        tick=Tick(7, 7 / 60),
    )

    assert frame.payload is bytes(frame.payload)
    assert frame.width_px == 1280
    assert frame.tick.step_index == 7


def test_png_rejects_jpeg_options_and_invalid_signatures() -> None:
    with pytest.raises(ValueError, match="do not accept JPEG options"):
        EncodedSensorRequest(_camera(), encoding="png")

    request = EncodedSensorRequest(
        _camera(),
        encoding="png",
        quality=None,
        chroma_subsampling=None,
    )
    with pytest.raises(ValueError, match="PNG signature"):
        EncodedSensorFrame(
            handle=request.handle,
            payload=b"not-png",
            width_px=1,
            height_px=1,
            encoding=request.encoding,
            quality=request.quality,
            color_space=request.color_space,
            chroma_subsampling=request.chroma_subsampling,
            tick=Tick(0, 0.0),
        )


def test_encoded_sensor_world_is_an_optional_runtime_protocol() -> None:
    class EncodedWorld:
        def read_encoded_sensors(self, requests):  # noqa: ANN001
            return tuple(requests)

        def step_and_read_articulations_and_encoded_sensors(
            self,
            articulation_handles,
            sensor_requests,
            count=1,
        ):  # noqa: ANN001
            del articulation_handles, sensor_requests, count
            return Tick(1, 1 / 60), (), ()

    assert isinstance(EncodedWorld(), EncodedSensorWorld)
