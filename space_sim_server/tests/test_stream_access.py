from space_arm_platform.stream_access import issue_stream_access_token, verify_stream_access_token


def test_stream_access_token_is_scoped_and_verifiable() -> None:
    token = issue_stream_access_token("test-secret", ["main", "wrist", "main"], ttl_seconds=60)
    payload = verify_stream_access_token("test-secret", token)
    assert payload["streamer_ids"] == ["main", "wrist"]
    assert payload["role"] == "operator"
