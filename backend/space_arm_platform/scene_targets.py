"""Stable scene identities and capture-target initial states (SI units).

No dynamics/CAD imports: shared by the API, model composer and simulator.
The original template remains available so old saved instances never load a
larger satellite at the legacy box's near-gripper initial position.
"""
from dataclasses import dataclass, replace
from pathlib import Path
import posixpath

LEGACY_TEMPLATE = "spacecraft-arm-teleop"
GROUND_TARGET_TEMPLATE = "sarm-ground-validation-grasp"
MESH_TARGET_TEMPLATE = "sarm-ground-validation-mesh-grasp"
SELF_COLLISION_TEMPLATE = "sarm-ground-validation-self-collision-grasp"
# Keep old scene identities/artifacts reproducible; triangle contact remains opt-in.
DEFAULT_TEMPLATE = SELF_COLLISION_TEMPLATE


@dataclass(frozen=True)
class CaptureTarget:
    model_file: str
    source_model: str
    position_m: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]
    hinge_body: str = ""
    hinge_joint: str = ""
    synthetic_mass_kg: float | None = None
    model_subdir: str = "."
    collision_model: str = "legacy"
    runtime_warning: str = ""

    @property
    def runtime_model(self) -> str:
        return posixpath.normpath(f"model/SARM/platform/{self.model_subdir}/{self.model_file}")

    def resolve_model(self, model_root: Path) -> Path:
        return (Path(model_root) / self.model_subdir / self.model_file).resolve()


TARGETS = {
    LEGACY_TEMPLATE: CaptureTarget(
        "sarm_platform.xml", "model/SARM/platform/sarm_platform.xml",
        (0.38754456, -0.00109359, 0.42397138),
        (0.99967109, 0.02433362, -0.00809494, 0.00024763),
    ),
    GROUND_TARGET_TEMPLATE: CaptureTarget(
        "sarm_ground_target.xml",
        "model/ground_validation_satellite/ground_validation_satellite_articulated.xml",
        (0.94, 0.039086, 0.40), (1.0, 0.0, 0.0, 0.0),
        "satellite_outer_panel", "outer_panel_hinge", 10.0,
        collision_model="coarse_boxes_no_target_self_collision",
    ),
}

TARGETS[MESH_TARGET_TEMPLATE] = replace(
    TARGETS[GROUND_TARGET_TEMPLATE],
    model_file="sarm_mesh_collision.xml",
    model_subdir="../../ground_validation_satellite/mesh_collision_trial",
    collision_model="original_triangle_rigid_flex",
    runtime_warning="高精度三角网格碰撞为实验配置：铰链零位已有接触，运行速度显著低于实时；未完成机构与长时稳定性验收。",
)


TARGETS[SELF_COLLISION_TEMPLATE] = replace(
    TARGETS[GROUND_TARGET_TEMPLATE],
    model_file="sarm_ground_target_self_collision.xml",
    collision_model="coarse_boxes_with_target_self_collision",
    runtime_warning="内部接触使用粗碰撞体，铰链附近留有近似间隙；接触角度不代表真实机构限位。",
)


def capture_target(template_id: str) -> CaptureTarget:
    try:
        return TARGETS[template_id]
    except KeyError as error:
        raise ValueError(f"unsupported scene template: {template_id}") from error
