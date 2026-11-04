"""Inventory actual local model components without inventing readiness."""
from pathlib import Path
import xml.etree.ElementTree as ET
from .catalog import TASK_MODEL, FLIGHT_MODEL
from .manifest import digest

MODEL_CANDIDATES = (
    ("service_with_task_box", "model/SARM/platform/sarm_20261012.xml", "操作星、单臂和原尺寸任务盒"),
    ("service_dual_arm", "model/SARM/platform/sarm_dual_capture26.xml", "操作星与独立捕获臂"),
    ("target_ground", "model/ground_validation_satellite/ground_validation_satellite_articulated.xml", "带铰接太阳翼的地面验证星"),
    ("target_lro", "model/nasa_lro_a/nasa_lro_a.xml", "LRO 外形资产"),
    ("arm_delivered", "model/SARM_mjcf_20261012/SARM_convex.xml", "交付机械臂凸分解模型"),
    ("task_box_module", "model/SARM/platform/sarm_task_box_module.xml", "任务盒装配模型"),
)


def inventory(repository):
    repository = Path(repository).resolve()
    records = []
    for relative in (TASK_MODEL, FLIGHT_MODEL):
        model = repository / relative
        root = ET.parse(model).getroot()
        parents = {c: p for p in root.iter() for c in p}
        names = ["cubesat_bus", "capture_target", "link1", "task_box_base", "drawer_base",
                 "aviation_plug", "guide_plug", "cylindrical_plug", "locking_rod"]
        for name in names:
            node = root.find(f'.//body[@name="{name}"]')
            if node is None:
                continue
            geoms = list(node.iter("geom"))
            inertial = node.find("inertial")
            bodies = {b.get('name') for b in node.iter('body')}
            joints = [dict(j.attrib, type='free' if j.tag == 'freejoint' else j.get('type', 'hinge'))
                      for j in node.iter() if j.tag in ('joint', 'freejoint')]
            records.append(dict(id=f"{model.stem}:{name}", model=relative,
                                model_sha256=digest(model), body=name,
                                parent_body=parents[node].get("name"),
                                position=node.get("pos", "0 0 0"), quaternion=node.get("quat", "1 0 0 0"),
                                units="m_kg_s", geom_count=len(geoms),
                                visual_geoms=sum(g.get("group") == "2" for g in geoms),
                                collision_note="Actual compiled defaults and contacts require native verification",
                                joint_names=[j.get('name') for j in joints], joint_definitions=joints,
                                collision_exclusions=[dict(e.attrib) for e in root.findall('./contact/exclude')
                                    if e.get('body1') in bodies or e.get('body2') in bodies],
                                site_names=[s.get("name") for s in node.iter("site")],
                                explicit_body_inertial=None if inertial is None else dict(inertial.attrib),
                                readiness="source_present", independently_extractable=False))
    candidates = []
    for key, relative, label in MODEL_CANDIDATES:
        path = repository / relative
        entry = dict(id=key, model=relative, label=label, available=path.is_file(), accepted=False)
        if path.is_file():
            root = ET.parse(path).getroot()
            entry.update(model_sha256=digest(path), readiness="source_present_not_integrated_variant",
                         bodies=[b.get('name') for b in root.iter('body') if b.get('name')],
                         includes=[n.get('file') for n in root.iter('include')],
                         reuse_requires=["resolved_assets_and_plugins", "mount_frames", "collision_and_inertia_audit",
                                         "controller_compatibility", "native_task_validation"])
        candidates.append(entry)
    return dict(schema="space-arm-asset-registry/1", assets=records, model_candidates=candidates,
                missing_phase_one_capabilities=['detachable_drawer_and_receiver_collision_model',
                                                'physical_docking_and_continuous_handoff'],
                missing_phase_two_assets=["free_drawer_and_receiver", "lidded_stowage_bin",
                    "irregular_rocks", "scissors_and_cuttable_wire_in_this_workspace", "refueling_interface_and_flow_model"],
                pairing_rules=[dict(family=kind, plug=f"{kind}_plug", pickup=f"{kind}_pickup",
                                    receivers=[f"{kind}_insert_40p1", f"{kind}_insert_40p5"])
                               for kind in ("aviation", "guide", "cylindrical")],
                extraction_note="Components still share defaults, meshes, constraints and plugins; do not export body nodes alone")
