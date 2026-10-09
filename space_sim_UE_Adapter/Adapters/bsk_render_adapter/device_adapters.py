"""Automatic Basilisk device-to-render visual adapters.

The adapters consume the same public configuration objects and output messages
used by Basilisk's Vizard interface.  They never participate in dynamics.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

import numpy as np

from .descriptors import VisualElement


def _items(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return list(value)
    return [value]


def _label(value: Any, fallback: str) -> str:
    raw = getattr(value, "label", "")
    if isinstance(raw, bytes):
        raw = raw.split(b"\0", 1)[0].decode("utf-8", errors="replace")
    text = str(raw).strip("\0 ")
    return text or fallback


def _rgba255(values: Iterable[int] | None, default: tuple[float, ...]) -> list[float]:
    raw = list(values or [])
    if len(raw) < 3:
        return list(default)
    if len(raw) < 4:
        raw.append(255)
    return [max(0, min(255, int(item))) / 255.0 for item in raw[:4]]


def _safe_read(message: Any) -> Any | None:
    try:
        return message.read()
    except Exception:
        return None


def _vector3(values: Any, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float).reshape(-1)
    if result.size != 3 or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite values")
    return result


def reaction_wheel_visuals(effector: Any, parent_id: str, prefix: str | None = None) -> list[VisualElement]:
    """Discover every wheel in a ``ReactionWheelStateEffector``."""

    configs = list(getattr(effector, "ReactionWheelData", []))
    messages = list(getattr(effector, "rwOutMsgs", []))
    count = max(len(configs), len(messages))
    result: list[VisualElement] = []
    base = prefix or f"{parent_id}/reaction_wheel"
    for index in range(count):
        config = configs[index] if index < len(configs) else _safe_read(messages[index])
        message = messages[index] if index < len(messages) else None
        if config is None:
            continue
        position = _vector3(getattr(config, "rWB_B", [0.0, 0.0, 0.0]), "rWB_B").tolist()
        axis = _vector3(getattr(config, "gsHat_B", [0.0, 0.0, 1.0]), "gsHat_B").tolist()
        omega_max = float(getattr(config, "Omega_max", -1.0))
        torque_max = float(getattr(config, "u_max", -1.0))
        integrated = {
            "time_ns": None,
            "angle_rad": float(getattr(config, "theta", 0.0)),
            "omega_rad_s": float(getattr(config, "Omega", 0.0)),
        }

        def state_provider(sim_time_ns=None, output=message, fallback=config, integration=integrated):
            state = _safe_read(output) if output is not None else fallback
            omega = float(getattr(state, "Omega", 0.0))
            if sim_time_ns is not None:
                previous_time = integration["time_ns"]
                if previous_time is None:
                    integration["angle_rad"] = float(getattr(state, "theta", integration["angle_rad"]))
                elif sim_time_ns >= previous_time:
                    delta_seconds = (sim_time_ns - previous_time) * 1.0e-9
                    integration["angle_rad"] += 0.5 * (integration["omega_rad_s"] + omega) * delta_seconds
                elif previous_time is not None:
                    integration["angle_rad"] = float(getattr(state, "theta", 0.0))
                integration["time_ns"] = sim_time_ns
                integration["omega_rad_s"] = omega
            angle = float(integration["angle_rad"])
            torque = float(getattr(state, "u_current", 0.0))
            speed_limit = float(getattr(state, "Omega_max", omega_max))
            torque_limit = float(getattr(state, "u_max", torque_max))
            saturated = speed_limit > 0.0 and abs(omega) >= speed_limit
            return {
                "visible": True,
                "value": omega,
                "channels": {
                    "angle_rad": angle,
                    "omega_rad_s": omega,
                    "torque_Nm": torque,
                    "omega_max_rad_s": speed_limit,
                    "torque_max_Nm": torque_limit,
                    "saturated": saturated,
                    "enabled": True,
                },
            }

        result.append(
            VisualElement(
                visual_id=f"{base}/{index}",
                kind="reaction_wheel",
                parent_id=parent_id,
                position_body_m=position,
                normal_body=axis,
                size_m=0.30,
                range_m=0.12,
                color_rgba=(0.25, 0.55, 1.0, 1.0),
                label=_label(config, f"RW{index + 1}"),
                properties={"device_index": index, "state_provider_accepts_sim_time": True},
                channel_schema={
                    "angle_rad": {"type": "number", "unit": "rad"},
                    "omega_rad_s": {"type": "number", "unit": "rad/s"},
                    "torque_Nm": {"type": "number", "unit": "N*m"},
                    "omega_max_rad_s": {"type": "number", "unit": "rad/s"},
                    "torque_max_Nm": {"type": "number", "unit": "N*m"},
                    "saturated": {"type": "boolean"},
                    "enabled": {"type": "boolean"},
                },
                state_provider=state_provider,
            )
        )
    return result


def mjscene_reaction_wheel_visuals(
    scene: Any,
    wheel_definitions: Iterable[dict[str, Any]],
    parent_id: str,
    prefix: str | None = None,
) -> list[VisualElement]:
    """Describe reaction wheels implemented as native MJScene hinge bodies.

    Each definition requires ``body_name`` and ``joint_name``. Optional fields
    provide the body-local mount, spin axis, dimensions, limits, label, and the
    final ``SingleActuatorMsg`` command consumed by the MuJoCo motor. The
    renderer reads joint position/rate directly; it never integrates a second
    wheel state.
    """

    result: list[VisualElement] = []
    base = prefix or f"{parent_id}/reaction_wheel"
    for index, raw in enumerate(wheel_definitions):
        definition = dict(raw)
        body_name = str(definition["body_name"])
        joint_name = str(definition["joint_name"])
        joint = scene.getBody(body_name).getScalarJoint(joint_name)
        position = _vector3(definition.get("position_body_m", (0.0, 0.0, 0.0)), "position_body_m").tolist()
        axis = _vector3(definition.get("axis_body", (0.0, 0.0, 1.0)), "axis_body")
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm <= 0.0:
            raise ValueError("axis_body must be non-zero")
        axis = (axis / axis_norm).tolist()
        command_message = definition.get("command_message")
        omega_max = float(definition.get("omega_max_rad_s", -1.0))
        torque_max = float(definition.get("torque_max_Nm", -1.0))

        def state_provider(
            position_message=joint.stateOutMsg,
            rate_message=joint.stateDotOutMsg,
            torque_message=command_message,
            speed_limit=omega_max,
            command_limit=torque_max,
        ):
            position_state = _safe_read(position_message)
            rate_state = _safe_read(rate_message)
            torque_state = _safe_read(torque_message) if torque_message is not None else None
            angle = float(getattr(position_state, "state", 0.0))
            omega = float(getattr(rate_state, "state", 0.0))
            torque = float(getattr(torque_state, "input", 0.0))
            return {
                "visible": True,
                "value": omega,
                "channels": {
                    "angle_rad": angle,
                    "omega_rad_s": omega,
                    "torque_Nm": torque,
                    "omega_max_rad_s": speed_limit,
                    "torque_max_Nm": command_limit,
                    "saturated": speed_limit > 0.0 and abs(omega) >= speed_limit,
                    "enabled": True,
                },
            }

        result.append(
            VisualElement(
                visual_id=f"{base}/{index}",
                kind="reaction_wheel",
                parent_id=parent_id,
                position_body_m=position,
                normal_body=axis,
                size_m=float(definition.get("diameter_m", 0.05)),
                range_m=float(definition.get("thickness_m", 0.024)),
                color_rgba=definition.get("color_rgba", (0.25, 0.55, 1.0, 1.0)),
                label=str(definition.get("label", body_name)),
                properties={"device_index": index, "dynamics_source": "mjscene_joint"},
                channel_schema={
                    "angle_rad": {"type": "number", "unit": "rad"},
                    "omega_rad_s": {"type": "number", "unit": "rad/s"},
                    "torque_Nm": {"type": "number", "unit": "N*m"},
                    "omega_max_rad_s": {"type": "number", "unit": "rad/s"},
                    "torque_max_Nm": {"type": "number", "unit": "N*m"},
                    "saturated": {"type": "boolean"},
                    "enabled": {"type": "boolean"},
                },
                state_provider=state_provider,
            )
        )
    return result


def thruster_visuals(
    effectors: Any,
    parent_id: str,
    *,
    colors: Any = None,
    prefix: str | None = None,
) -> list[VisualElement]:
    """Discover thrusters in one or more Basilisk thruster effectors."""

    result: list[VisualElement] = []
    base = prefix or f"{parent_id}/thruster"
    if isinstance(colors, (list, tuple)) and colors and all(isinstance(item, (int, float)) for item in colors):
        color_sets = [colors]
    else:
        color_sets = _items(colors)
    device_index = 0
    for cluster_index, effector in enumerate(_items(effectors)):
        configs = list(getattr(effector, "thrusterData", []))
        messages = list(getattr(effector, "thrusterOutMsgs", []))
        offset = np.asarray(getattr(effector, "r_PcP_P", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
        cluster_color = color_sets[cluster_index] if cluster_index < len(color_sets) else None
        color = _rgba255(cluster_color, (1.0, 0.35, 0.05, 0.85))
        for local_index in range(max(len(configs), len(messages))):
            config = configs[local_index] if local_index < len(configs) else _safe_read(messages[local_index])
            message = messages[local_index] if local_index < len(messages) else None
            if config is None:
                continue
            location = np.asarray(
                getattr(config, "thrLoc_B", getattr(config, "thrusterLocation", [0.0, 0.0, 0.0])),
                dtype=float,
            ).reshape(3) + offset
            thrust_direction = _vector3(
                getattr(config, "thrDir_B", getattr(config, "thrusterDirection", [1.0, 0.0, 0.0])),
                "thruster direction",
            )
            max_thrust = float(getattr(config, "MaxThrust", getattr(config, "maxThrust", 0.0)))

            def state_provider(output=message, fallback=config, configured_max=max_thrust):
                state = _safe_read(output) if output is not None else fallback
                thrust = float(getattr(state, "thrustForce", 0.0))
                limit = float(getattr(state, "maxThrust", configured_max))
                throttle = max(0.0, min(1.0, thrust / limit)) if limit > 0.0 else max(0.0, thrust)
                return {
                    "visible": thrust > 1.0e-9,
                    "value": throttle,
                    "channels": {
                        "thrust_N": thrust,
                        "max_thrust_N": limit,
                        "throttle": throttle,
                        "enabled": thrust > 1.0e-9,
                    },
                }

            result.append(
                VisualElement(
                    visual_id=f"{base}/{device_index}",
                    kind="thruster",
                    parent_id=parent_id,
                    position_body_m=location.tolist(),
                    normal_body=(-thrust_direction).tolist(),
                    field_of_view_rad=(0.35,),
                    size_m=0.08,
                    range_m=1.5,
                    color_rgba=color,
                    label=_label(config, f"THR{device_index + 1}"),
                    properties={"device_index": device_index, "cluster_index": cluster_index},
                    channel_schema={
                        "thrust_N": {"type": "number", "unit": "N"},
                        "max_thrust_N": {"type": "number", "unit": "N"},
                        "throttle": {"type": "number", "minimum": 0.0, "maximum": 1.0},
                        "enabled": {"type": "boolean"},
                    },
                    state_provider=state_provider,
                )
            )
            device_index += 1
    return result


def css_visuals(devices: Any, parent_id: str, prefix: str | None = None) -> list[VisualElement]:
    """Discover coarse sun sensors and their live configuration messages."""

    result: list[VisualElement] = []
    base = prefix or f"{parent_id}/css"
    for index, sensor in enumerate(_items(devices)):
        message = getattr(sensor, "cssConfigLogOutMsg", None)
        max_output = float(getattr(sensor, "maxOutput", -1.0))

        def state_provider(source=sensor, output=message, configured_max=max_output):
            state = _safe_read(output)
            if state is not None:
                signal = float(getattr(state, "signal", 0.0))
                maximum = float(getattr(state, "maxSignal", configured_max))
                minimum = float(getattr(state, "minSignal", -1.0))
            else:
                data_message = getattr(source, "cssDataOutMsg", None)
                data = _safe_read(data_message)
                signal = float(getattr(data, "OutputData", 0.0))
                maximum = configured_max
                minimum = 0.0
            normalized = max(0.0, min(1.0, signal / maximum)) if maximum > 0.0 else max(0.0, signal)
            valid = signal >= max(minimum, 0.0)
            return {
                "visible": True,
                "value": signal,
                "channels": {
                    "signal": signal,
                    "normalized_signal": normalized,
                    "valid": valid,
                    "enabled": True,
                },
            }

        result.append(
            VisualElement(
                visual_id=f"{base}/{index}",
                kind="css",
                parent_id=parent_id,
                position_body_m=_vector3(getattr(sensor, "r_B", [0.0, 0.0, 0.0]), "CSS r_B").tolist(),
                normal_body=_vector3(getattr(sensor, "nHat_B", [1.0, 0.0, 0.0]), "CSS nHat_B").tolist(),
                field_of_view_rad=(float(getattr(sensor, "fov", np.pi / 2.0)),),
                size_m=0.05,
                range_m=1.5,
                color_rgba=(1.0, 0.7, 0.05, 0.32),
                label=str(getattr(sensor, "ModelTag", "") or f"CSS{index + 1}"),
                properties={"device_index": index},
                channel_schema={
                    "signal": {"type": "number"},
                    "normalized_signal": {"type": "number", "minimum": 0.0, "maximum": 1.0},
                    "valid": {"type": "boolean"},
                    "enabled": {"type": "boolean"},
                },
                state_provider=state_provider,
            )
        )
    return result
