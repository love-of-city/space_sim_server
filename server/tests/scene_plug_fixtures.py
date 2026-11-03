"""Measured scene-library plug fixtures, separate from collection v17 defaults."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from simulation.scripted_tasks.scene_plug import PlugInsertTask, PlugInsertConfig, PlugSample
from simulation.scripted_tasks.plug_profiles import SCENE_PLUG_PROFILES as PLUG_PROFILES
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG
MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"

def task_sample():
    task = PlugInsertTask(MODEL)
    q = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0.0, 0.0]
    sample = PlugSample(
        0.0,
        q,
        np.zeros(8),
        np.array([0.3695, 0.33003, 0.032]),
        np.diag([1.0, -1.0, -1.0]),
        np.zeros(3),
        np.zeros(3),
        np.zeros(6),
        True,
    )
    task.step(sample)
    return task, sample

def make_task(name):
    profile = PLUG_PROFILES[name]
    task = PlugInsertTask(MODEL, PlugInsertConfig(plug_body=profile.body))
    body = ET.parse(MODEL).getroot().find(f'./worldbody/body[@name="{profile.body}"]')
    q = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0.0, 0.0]
    sample = PlugSample(
        0.0,
        q,
        np.zeros(8),
        np.fromstring(body.get("pos"), sep=" "),
        quaternion_wxyz_to_matrix(np.fromstring(body.get("quat"), sep=" ")),
        np.zeros(3),
        np.zeros(3),
        np.zeros(6),
        True,
    )
    task.step(sample)
    return task, sample
