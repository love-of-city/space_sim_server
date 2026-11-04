"""Small, named adapters for MuJoCo force/torque pairs (no force equations)."""

from dataclasses import dataclass


@dataclass(frozen=True)
class WrenchSensor:
    name: str
    site: str
    force_sensor: str
    torque_sensor: str
    parent_body: str
    child_body: str

    def observation(self, stepper, observation_time_ns: int):
        reading = stepper.read_wrench(self.force_sensor, self.torque_sensor)
        if reading is None:
            return None
        values, sample_ns, end_ns = reading
        valid = sample_ns is not None
        if valid and not 0 <= sample_ns < end_ns == observation_time_ns:
            raise ValueError(
                "wrench timestamp is outside the completed physics interval"
            )
        return {
            "sensor_name": self.name,
            "site_name": self.site,
            "parent_body": self.parent_body,
            "child_body": self.child_body,
            "frame": "sensor_site",
            "sign_convention": "mujoco_force_torque",
            "measurement": "raw_interface_wrench",
            "sample_phase": "last_substep_start",
            "valid": valid,
            "sample_time_ns": str(sample_ns if valid else observation_time_ns),
            "interval_end_time_ns": str(end_ns if valid else observation_time_ns),
            "force_n": values[:3].tolist(),
            "torque_nm": values[3:].tolist(),
        }


WRIST_SENSOR = WrenchSensor(
    "wrist_ft",
    "wrist_ft_site",
    "wrist_ft_force",
    "wrist_ft_torque",
    "ee_base",
    "wrist_ft_tool",
)
