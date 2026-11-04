"""Design catalogue in source order; runtime support and acceptance are separate."""
from copy import deepcopy

OPERATIONS = [
    ("drawer_cycle", "抽屉拉起与放回"),
    ("aviation_plug_insert", "航空插头拔出与插入"),
    ("guide_plug_insert", "导向插头拔出与插入"),
    ("cylindrical_plug_insert", "圆柱插头拔出与插入"),
    ("locking_rod_cycle", "杆状螺丝放倒、复位与锁紧"),
]
TASK_MODEL = "model/SARM/platform/sarm_20261012.xml"
FLIGHT_MODEL = "model/SARM/platform/sarm_ground_target_self_collision.xml"
FAMILIES = [
    ("01_self_operation", "本星任务盒操作", OPERATIONS,
     ["service_bus", "single_arm", "service_task_box"]),
    ("02_docked_operation", "对接后操作靶星任务盒", OPERATIONS,
     ["service_bus", "operating_arm", "capture_arm", "preheld_target", "target_bus", "target_task_box"]),
    ("03_rendezvous", "交会目标卫星", [("rendezvous", "交会目标卫星")],
     ["service_bus", "target_bus", "relative_navigation"]),
    ("04_cross_transfer", "跨星转移安装", [
        ("drawer_cycle", "从靶星取出抽屉并装入操作星"),
        ("aviation_plug_insert", "从靶星拔出航空插头并插入操作星"),
        ("guide_plug_insert", "从靶星拔出导向插头并插入操作星"),
        ("cylindrical_plug_insert", "从靶星拔出圆柱插头并插入操作星")],
     ["service_bus", "operating_arm", "capture_arm", "preheld_target", "target_bus", "donor_module", "receiver_module"]),
    ("05_grasp_observe", "单臂抓取与手眼观测", [
        ("solar_wing", "抓取靶星太阳翼"), ("floating_connectors", "抓取漂浮螺栓及三类插头"),
        ("floating_drawer", "抓取漂浮抽屉"), ("floating_components", "抓取漂浮光敏、天线"),
        ("floating_rock", "抓取漂浮不规则石头"), ("wrist_observe", "手眼相机拍摄目标组件")],
     ["service_bus", "single_arm", "wrist_camera", "target_object"]),
    ("06_dual_arm", "双臂收纳与拼接", [
        ("stow_connectors", "螺栓及插头收纳"), ("stow_components", "光敏、天线收纳"),
        ("stow_rock", "不规则石头收纳"), ("assemble", "一臂持工件、另一臂持底座并拼接")],
     ["service_bus", "dual_arm", "workpiece", "lidded_bin_or_mating_base"]),
    ("07_cut", "线缆剪切", [("cut_wire", "剪断靶星线缆")],
     ["service_bus", "single_arm", "scissors", "target_bus", "wire"]),
    ("08_refuel", "加注作业", [("refuel", "用加油头加注")],
     ["service_bus", "single_arm", "nozzle", "target_bus", "fuel_port"]),
]


def catalog():
    scenes = []
    for index, (family, label, operations, roles) in enumerate(FAMILIES, 1):
        for task, task_label in operations:
            adapter = "task_box" if index == 1 and task != "locking_rod_cycle" else (
                "flight" if index == 3 else None)
            gaps = []
            if index == 1 and task == "locking_rod_cycle":
                gaps += ["current_rod_independent_scoring_adapter"]
            if index in (2, 4):
                gaps += ["dual_arm_preheld_target_and_hold_validation", "target_owned_task_modules"]
            if index == 4:
                gaps += ["cross_body_transfer_and_independent_scoring"]
                if task == "drawer_cycle":
                    gaps += ["detachable_drawer_and_receiver"]
            if index > 4:
                gaps += ["phase_two_task_adapter_and_validation"]
            scenes.append(dict(id=f"{family}/{task}", family=family, family_number=index,
                               label=label, task_label=task_label, task=task, roles=roles,
                               phase=1 if index <= 4 else 2, adapter=adapter,
                               source_model=(FLIGHT_MODEL if index == 3 else
                                   'model/SARM/platform/sarm_dual_capture26.xml' if index in (2, 4) else TASK_MODEL),
                               readiness="native_runner_available" if adapter else "specified",
                               missing_capabilities=gaps, accepted=False))
    return dict(schema="space-arm-scene-catalog/1", scenes=deepcopy(scenes),
                phase_one_scope=phase_one_scope(),
                fixture_runner_scene_ids=[s['id'] for s in scenes if s['family_number'] in (2, 4)
                    and s['id'] != '04_cross_transfer/drawer_cycle'],
                assumptions={"cross_transfer_destination": "service_satellite",
                             "aviation_and_space_connector": "same_family_pending_confirmation"},
                scaling_policy={"satellite_uniform_scaling_allowed": True,
                                "task_interface_dimensions_preserved": True,
                                "mass_rule": "explicit_constant_density_or_fixed_mass_required"})


def recipe(scene_id):
    for item in catalog()["scenes"]:
        if item["id"] == scene_id:
            return item
    raise ValueError(f"Unknown scene id: {scene_id}")


def phase_one_scope():
    """Original design scope; unsupported adapters remain explicitly unavailable."""
    return dict(version='local-phase-one/3-dual-arm-preheld',
                connected_initial_state_families=[2, 4],
                connected_initial_state='target_held_by_capture_arm_at_working_distance',
                capture_arm_holds_during_operating_arm_motion=True,
                target_mounted_on_service_bus_allowed=False,
                deferred_scene_ids=['04_cross_transfer/drawer_cycle'],
                required_tasks=14,
                physical_docking_required=False,
                rendezvous_scope='far_field_rendezvous_and_hold',
                rod_scope='source_model_minus_0p5_rad_cycle_restore_and_lock',
                ue_visual_and_linux_acceptance=False)
