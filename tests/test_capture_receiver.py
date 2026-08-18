from space_arm_platform.capture_receiver import CaptureReceiver


def test_capture_receiver_separates_preview_from_authoritative_data() -> None:
    recorded: list[tuple[dict, dict]] = []
    receiver = CaptureReceiver("127.0.0.1", 0, lambda metadata, products: recorded.append((metadata, products)))

    receiver.route_capture(
        {
            "stream_kind": "preview",
            "state_kind": "presentation",
            "camera_id": "overview",
            "products": [{"name": "rgb", "file_name": "preview.jpg"}],
        },
        {"rgb": b"JPEG"},
    )
    assert recorded == []
    assert receiver.latest("overview").data == b"JPEG"

    metadata = {
        "stream_kind": "authoritative",
        "state_kind": "authoritative",
        "camera_id": "overview",
        "source_frame_id": "4",
        "sim_time_ns": "5",
    }
    receiver.route_capture(metadata, {"rgb": b"PNG", "depth": b"EXR"})
    assert recorded == [(metadata, {"rgb": b"PNG", "depth": b"EXR"})]
    assert receiver.latest("overview").data == b"JPEG"
