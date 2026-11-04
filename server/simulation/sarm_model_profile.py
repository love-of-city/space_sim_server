"""Model-specific control names; passive linkage joints never become servo axes."""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

@dataclass(frozen=True)
class SarmModelProfile:
    linkage: bool
    joints: tuple
    arm_actuators: tuple
    gripper_actuator: str = ''

    @property
    def joint_names(self):
        return tuple(joint for _, joint in self.joints)

def load_model_profile(model_path: Path) -> SarmModelProfile:
    root = ET.parse(model_path).getroot()
    linkage = root.find('./actuator/position[@name="ee_fingers_position"]') is not None
    if linkage:
        return SarmModelProfile(True,
            tuple((f'link{i}', f'joint{i}') for i in range(1,6)) +
            (('ee_base','joint6'), ('ee_link11','ee_link11_joint'), ('ee_link12','ee_link12_joint')),
            tuple(f'joint{i}_position' for i in range(1,7)), 'ee_fingers_position')
    return SarmModelProfile(False,
        tuple((f'link{i}', f'joint{i}') for i in range(1,7)) +
        (('finger1','joint_finger1'), ('finger2','joint_finger2')),
        tuple(f'sarm_joint{i}_motor' for i in range(1,7)))
