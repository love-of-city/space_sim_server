"""Versioned physical task requirements, independent of any expert/policy."""
from dataclasses import dataclass


@dataclass(frozen=True)
class DrawerBenchmarkCriteria:
    task_id: str = "drawer_cycle/full_cycle_hold_v1"
    lift_m: float = .08
    loaded_position_tolerance_m: float = .005
    close_tolerance_m: float = .002
    buckle_tolerance_rad: float = .025
    position_tolerance_m: float = .003  # Legacy hover error is diagnostic only.
    clearance_m: float = .205          # Legacy hover point is diagnostic only.
    grip_offset_m: tuple = (0., 0., -.005)  # Delivered handle calibration.
    hold_s: float = 2.
    verify_hold_s: float = 2.
    retreat_distance_m: float = .09
    instruction: str = (
        "将抽屉抽出至少75毫米并保持至少2秒，然后推回关闭；"
        "松开夹爪并撤离把手至少90毫米，保持抽屉关闭、卡扣复位至少2秒。"
    )
