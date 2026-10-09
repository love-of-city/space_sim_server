import socket

from space_arm_platform.protocol import decode_packet, encode_packet, recv_socket


def test_length_prefixed_json_round_trip() -> None:
    message = {"protocol": "space-arm-control/1", "type": "action", "server_sequence": "9007199254740993"}
    packet = encode_packet(message)
    assert decode_packet(packet) == message


def test_socket_receiver_accepts_fragmented_packet() -> None:
    left, right = socket.socketpair()
    try:
        packet = encode_packet({"type": "observation", "sim_time_ns": "123456789012345678"})
        for byte in packet:
            left.send(bytes([byte]))
        assert recv_socket(right)["sim_time_ns"] == "123456789012345678"
    finally:
        left.close()
        right.close()

