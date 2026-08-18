import asyncio

from space_arm_platform.app import OperatorSessions


class FakeWebSocket:
    def __init__(self) -> None:
        self.accepted = False

    async def accept(self) -> None:
        self.accepted = True


def test_viewer_is_identified_for_promotion_when_owner_disconnects() -> None:
    async def scenario() -> None:
        sessions = OperatorSessions()
        first = FakeWebSocket()
        second = FakeWebSocket()
        first_id, first_granted = await sessions.connect(first)  # type: ignore[arg-type]
        second_id, second_granted = await sessions.connect(second)  # type: ignore[arg-type]
        assert first_granted
        assert not second_granted
        released, promoted = await sessions.disconnect(first_id)
        assert released
        assert promoted == (second_id, second)
        assert sessions.is_owner(second_id)

    asyncio.run(scenario())
