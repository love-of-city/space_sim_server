"""Episode-scoped floating instances; the existing target remains the controller goal."""
from copy import deepcopy
import xml.etree.ElementTree as ET
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .floating_assets import FLOATING_TASKS
from simulation.socket_colors import COLOR_KEYS


class FloatingSceneConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    object_count: int = Field(default=1, ge=1, le=len(COLOR_KEYS), strict=True)
    composition: Literal["different-types", "same-type-colors"] = "different-types"
    distractor_types: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_selection(self):
        if len(set(self.distractor_types)) != len(self.distractor_types) or any(t not in FLOATING_TASKS for t in self.distractor_types):
            raise ValueError("Distractor types must be unique registered floating tasks")
        if self.composition == "same-type-colors":
            if self.distractor_types:
                raise ValueError("same-type-colors uses the target type, not a distractor type pool")
        return self


def target_variation_options(scene):
    options = deepcopy(scene.get("floating_target", {}))
    if FloatingSceneConfig.model_validate(scene.get("floating_scene", {})).object_count > 1 and not options.get("color_choices"):
        options["color_choices"] = list(COLOR_KEYS)
    return options


def _free_joint(body):
    for child in body:
        if child.tag == "freejoint" or (child.tag == "joint" and child.get("type") == "free"):
            return child.get("name")
    raise ValueError("Floating instance requires a named free joint")


def apply_floating_objects(root, source, scene):
    from .floating_initialization import apply_floating_model, clone_floating_body, camera_in_tool, floating_release_options
    from .floating_variations import apply_target_variations
    from .randomization import generator
    from simulation.serial_chain_kinematics import SerialChainKinematics

    cfg = scene["collection_config"]
    options = FloatingSceneConfig.model_validate(scene.get("floating_scene", {}))
    task = scene.get("scripted_task_name") or cfg["task"]
    target = FLOATING_TASKS[task]
    release = cfg["floating_initial_state"]
    variation = cfg["floating_target_variation"]
    instances = [dict(instance_id=target.body, type_task=task, is_target=True,
                      free_joint=_free_joint(root.find(f'./worldbody/body[@name="{target.body}"]')),
                      initial_state=deepcopy(release), variation=deepcopy(variation))]
    rng = generator(scene.get("seed", 0), "floating-objects-selection-v1")
    if options.composition == "different-types":
        pool = [t for t in (options.distractor_types or tuple(FLOATING_TASKS)) if t != task]
        if options.object_count-1 > len(pool):
            raise ValueError("Not enough distinct non-target types in the requested pool")
        types = [str(t) for t in rng.choice(pool, size=options.object_count-1, replace=False)]
    else:
        types = [task] * (options.object_count-1)
    palette = [c for c in COLOR_KEYS if c != variation["color"]]
    colors = [str(c) for c in rng.choice(palette, size=len(types), replace=False)]
    if types:
        original = ET.parse(source).getroot()
        chain = SerialChainKinematics.from_mjcf(source, base_body="cubesat_bus",
                    joint_names=tuple(f"joint{i}" for i in range(1,7)), tool_site="sarm_ee")
        _, hand_rotation = chain.forward(np.asarray(scene["randomization"]["arm_joint_position_rad"][:6]))
        _, camera_local_rotation = camera_in_tool(original)
        camera_rotation = hand_rotation @ camera_local_rotation
        camera = np.asarray(release["camera_position_body_m"])
        def view(point):
            local = camera_rotation.T@(np.asarray(point)-camera)
            return local[:2]/(-local[2])
        occupied = [np.asarray(release["grasp_position_body_m"])]
        pose_options = floating_release_options(scene).snapshot()
        for index, (kind, color) in enumerate(zip(types, colors), 1):
            instance_id = f"floating_distractor_{index:02d}"
            asset = FLOATING_TASKS[kind]
            pose_rng = generator(scene.get("seed", 0), f"floating-object-pose-v1/{index}")
            for attempt in range(128):
                child = dict(seed=int(pose_rng.integers(0, 2**31)), scripted_task_name=kind,
                             floating_capture=pose_options, randomization=deepcopy(scene["randomization"]),
                             collection_config={"task": kind})
                try:
                    state = apply_floating_model(deepcopy(original), source, child)
                except ValueError as error:
                    if "No reachable floating release" in str(error):
                        continue
                    raise
                point = np.asarray(state["grasp_position_body_m"])
                if all(np.linalg.norm(point-other) >= .12 and np.linalg.norm(view(point)-view(other)) >= .18 for other in occupied):
                    break
            else:
                raise ValueError(f"Cannot place {instance_id} with reachable, separated initial views")
            occupied.append(point)
            body = clone_floating_body(original, asset.source_body or asset.body, instance_id)
            for key, values in (("pos", state["position_body_m"]), ("quat", state["orientation_wxyz"])):
                body.set(key, " ".join(format(float(v), ".17g") for v in values))
            root.find("worldbody").append(body)
            variant_scene = dict(seed=child["seed"], floating_target=dict(color_choices=[color]), collection_config={})
            realized = apply_target_variations(root, variant_scene, instance_id, instance_key=instance_id)
            state = deepcopy(state)
            state["target_body"] = instance_id
            instances.append(dict(instance_id=instance_id, type_task=kind, is_target=False,
                                  free_joint=_free_joint(body), initial_state=state, variation=realized))
    record = dict(schema="floating-objects/1", options=options.model_dump(mode="json"),
                  target_instance_id=target.body, instances=instances)
    if "floating_objects" in cfg and cfg["floating_objects"] != record:
        raise ValueError("Saved floating objects do not match source, options or sampled instances")
    cfg["floating_objects"] = record
    return record


def validate_floating_object_recording(cfg, steps, instruction):
    """Bind rendered instances, their observed identities and instruction to the target."""
    record = cfg["floating_objects"]
    assert record["schema"] == "floating-objects/1"
    options = FloatingSceneConfig.model_validate(record["options"])
    instances = record["instances"]
    assert len(instances) == options.object_count
    ids = [obj["instance_id"] for obj in instances]
    assert len(ids) == len(set(ids))
    targets = [obj for obj in instances if obj["is_target"]]
    assert len(targets) == 1 and targets[0]["instance_id"] == record["target_instance_id"]
    target = targets[0]
    assert target["initial_state"] == cfg["floating_initial_state"]
    assert target["variation"] == cfg["floating_target_variation"]
    types = [obj["type_task"] for obj in instances]
    colors = [obj["variation"]["color"] for obj in instances]
    if len(instances) > 1:
        assert all(color in COLOR_KEYS for color in colors) and len(set(colors)) == len(instances)
    if options.composition == "different-types":
        assert len(set(types)) == len(instances)
    else:
        assert len(set(types)) == 1
    kind, color = target["type_task"], target["variation"]["color"]
    matches = [obj for obj in instances if obj["type_task"] == kind and obj["variation"]["color"] == color]
    assert len(matches) == 1
    description = FLOATING_TASKS[kind].instruction_name
    if color:
        description = f"{color} {description}"
    assert instruction == f"Catch the {description}, hold it securely for two seconds, then bring it back above the workbench."
    for obs in steps:
        task = obs["scripted_task"]
        assert task["target_instance_id"] == record["target_instance_id"] == task["target_body"]
        assert not task["non_target_contacts"]
        actual = task["floating_objects"]
        assert [item["instance_id"] for item in actual] == ids
        for expected, item in zip(instances, actual):
            assert item["type_task"] == expected["type_task"] and item["color"] == expected["variation"]["color"]
            for key, size in (("position_body_m", 3), ("orientation_wxyz", 4), ("velocity_body_m_s", 3), ("angular_velocity_body_rad_s", 3)):
                values = np.asarray(item[key]);assert values.shape == (size,) and np.isfinite(values).all()
            assert abs(np.linalg.norm(item["orientation_wxyz"])-1) < 1e-6
            if not expected["is_target"]:
                assert not any(item["finger_contacts"]) and not any(item["finger_normal_force_n"])
