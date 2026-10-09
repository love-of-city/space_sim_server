"""Drive the actual Basilisk physics task on a drift-free rational time grid."""
from __future__ import annotations

from Basilisk.architecture import sysModel
from space_arm_platform.sampling import DYNAMICS_HZ, tick_time_ns


class RationalPhysicsClock(sysModel.SysModel):
    """Runs first in the physics task and sets its *next* actual interval.

    Basilisk ExecuteTaskList adds TaskPeriod after executing the task's models.
    Setting TaskPeriod here does not reschedule the in-progress invocation (unlike
    updatePeriod). MJScene integrates to CurrentSimNanos, so integration, output
    timestamps and divided IK/render work all use the same real task clock.
    """
    def __init__(self, task):
        super().__init__()
        self.ModelTag = "rational240HzPhysicsClock"
        self.task = task
        self.step_index = 0
        self._next_index = 0

    def Reset(self, CurrentSimNanos: int) -> None:
        if int(CurrentSimNanos) != 0:
            raise ValueError("physics clock reset requires a fresh simulation at time zero")
        self.step_index = self._next_index = 0
        self.task.TaskData.TaskPeriod = tick_time_ns(1, DYNAMICS_HZ)

    def UpdateState(self, CurrentSimNanos: int) -> None:
        now = int(CurrentSimNanos)
        self.step_index = self._next_index
        expected = tick_time_ns(self.step_index, DYNAMICS_HZ)
        if now != expected:
            raise RuntimeError(f"physics clock drift: step {self.step_index}, expected {expected}, got {now}")
        self._next_index += 1
        self.task.TaskData.TaskPeriod = tick_time_ns(self._next_index, DYNAMICS_HZ) - now
