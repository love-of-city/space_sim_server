"""Collection-time release velocities and controller tracking limits."""
from copy import deepcopy

from .floating_assets import FLOATING_TASKS
from .floating_capture import floating_controller_config
from .floating_initialization import floating_release_options
from .floating_objects import FloatingSceneConfig


def add_floating_options(parser):
    parser.add_argument("--object-count", type=int, help="Total floating instances including the target (default: 1)")
    parser.add_argument("--object-composition", choices=("different-types", "same-type-colors"))
    parser.add_argument("--distractor-types", choices=tuple(FLOATING_TASKS), nargs="+",
                        help="Eligible registered tasks for different-type distractors")
    for quantity, unit in (("speed", "m/s"), ("spin", "rad/s")):
        group = parser.add_mutually_exclusive_group()
        group.add_argument(f"--object-{quantity}", type=float, help=f"Fixed initial magnitude ({unit})")
        group.add_argument(f"--object-{quantity}-range", type=float, nargs=2,
                           metavar=("MIN", "MAX"), help=f"Uniform initial magnitude interval ({unit})")
    parser.add_argument("--retract-position", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="Final end-effector position in satellite body frame (m)")
    parser.add_argument("--retract-speed", type=float, help="Retraction linear speed limit (m/s)")
    parser.add_argument("--object-direction", choices=("random", "stationary", "approaching", "receding", "lateral_left", "lateral_right", "up", "down"))
    parser.add_argument("--spin-axis", choices=("random", "camera_x", "camera_y", "camera_z", "reference_z"))
    parser.add_argument("--spin-sign", type=int, choices=(-1, 1))
    for quantity, unit in (("linear", "m/s"), ("angular", "rad/s"), ("joint", "rad/s")):
        parser.add_argument(f"--tracking-{quantity}-speed", type=float, help=f"Controller tracking limit ({unit})")


def apply_floating_options(source, args, *, prepared=False):
    release, controller, retraction, objects = {}, {}, {}, {}
    for quantity, field in (("speed", "speed_m_s"), ("spin", "spin_rad_s")):
        fixed = getattr(args, f"object_{quantity}")
        interval = getattr(args, f"object_{quantity}_range")
        if fixed is not None:
            interval = (fixed, fixed)
        if interval is not None:
            release[f"min_{field}"], release[f"max_{field}"] = interval
    for argument, field in (("object_direction", "velocity_direction"), ("spin_axis", "spin_axis"), ("spin_sign", "spin_sign")):
        value = getattr(args, argument)
        if value is not None:
            release[field] = value
    for quantity, field in (("linear", "linear_speed_m_s"), ("angular", "angular_speed_rad_s"), ("joint", "arm_speed_rad_s")):
        value = getattr(args, f"tracking_{quantity}_speed")
        if value is not None:
            controller[field] = value
    for argument, field in (("retract_position", "position_body_m"), ("retract_speed", "speed_m_s")):
        value = getattr(args, argument)
        if value is not None:
            retraction[field] = value
    for argument, field in (("object_count", "object_count"), ("object_composition", "composition"), ("distractor_types", "distractor_types")):
        value = getattr(args, argument)
        if value is not None:
            objects[field] = value
    if release or controller or retraction or objects:
        if prepared:
            raise ValueError("Floating overrides cannot be combined with a prepared scene")
        if args.task not in FLOATING_TASKS:
            raise ValueError("Floating overrides require a floating capture task")
    result = deepcopy(source)
    if args.task in FLOATING_TASKS:
        result["floating_capture"] = {**result.get("floating_capture", {}), **release}
        result["floating_controller"] = {**result.get("floating_controller", {}), **controller}
        result["floating_retraction"] = {**result.get("floating_retraction", {}), **retraction}
        result["floating_scene"] = {**result.get("floating_scene", {}), **objects}
        FloatingSceneConfig.model_validate(result["floating_scene"])
        floating_release_options(result)
        floating_controller_config(args.task, result)
    return result
