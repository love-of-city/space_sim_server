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
    receiver.route_capture(metadata, {"rgb": b"PNG"})
    assert recorded == [(metadata, {"rgb": b"PNG"})]
    assert receiver.latest("overview").data == b"JPEG"


def test_capture_retry_after_lost_ack_is_idempotent():
    received=[]
    receiver=CaptureReceiver("127.0.0.1",0,lambda m,p:received.append(m))
    meta={"stream_kind":"authoritative","state_kind":"authoritative","ack_required":True,
          "session_id":"one","camera_id":"wrist","capture_sequence":"9"}
    receiver.route_capture(meta,{"rgb":b"jpg"})
    receiver.route_capture(meta,{"rgb":b"jpg"})
    assert len(received)==1
    assert receiver.authoritative_count==1


def test_network_ack_and_replay_with_real_socket():
    import json, socket, struct, time
    received=[]
    receiver=CaptureReceiver("127.0.0.1",0,lambda m,p:received.append((m,p)))
    receiver.start()
    try:
        for _ in range(100):
            if receiver._listener and receiver._listener.getsockname()[1]:break
            time.sleep(.01)
        meta={"protocol":"bsk-capture/1","stream_kind":"authoritative","state_kind":"authoritative",
              "ack_required":True,"session_id":"s","camera_id":"c","capture_sequence":"1",
              "products":[{"name":"rgb","blob_offset":"0","byte_length":"3"}]}
        data=json.dumps(meta).encode();payload=struct.pack('!I',len(data))+data+b"jpg"
        packet=struct.pack('!I',len(payload))+payload
        for _ in range(2):
            with socket.create_connection(receiver._listener.getsockname(),timeout=2) as sock:
                sock.sendall(packet)
                assert sock.recv(1)==b"\x01"
        assert receiver.wait_for_authoritative_idle()
        assert len(received)==1 and received[0][1]=={"rgb":b"jpg"}
    finally:receiver.close()
