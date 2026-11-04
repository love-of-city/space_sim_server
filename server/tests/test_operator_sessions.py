import asyncio

from space_arm_platform.app import OperatorSessions


class FakeWebSocket:
    def __init__(self) -> None:
        self.accepted = False

    async def accept(self) -> None:
        self.accepted = True


def user(identifier: str) -> dict[str, str]:
    return {"user_id": identifier, "username": identifier, "role": "operator"}


def test_new_page_automatically_replaces_active_page_when_activated() -> None:
    async def scenario() -> None:
        sessions = OperatorSessions()
        first = FakeWebSocket()
        second = FakeWebSocket()
        first_id, first_granted = await sessions.connect(first, user("same-user"))  # type: ignore[arg-type]
        second_id, second_granted = await sessions.connect(second, user("same-user"))  # type: ignore[arg-type]
        assert first_granted
        assert not second_granted
        previous = await sessions.activate(second_id)
        assert previous == (first_id, first)
        assert sessions.is_owner(second_id)
        assert await sessions.disconnect(second_id)
        assert not sessions.is_owner(first_id)

    asyncio.run(scenario())
