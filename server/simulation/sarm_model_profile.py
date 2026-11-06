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


@dataclass(frozen=True)
class ArmDescription:
    """Names belonging to one physical SARM operating arm."""
    prefix: str = ''

    def __post_init__(self):
        if self.prefix not in ('', 'left_', 'right_'):
            raise ValueError('Unsupported operating-arm prefix')

    @property
    def joints(self):
        return tuple((self.prefix+b, self.prefix+j) for b,j in (
            *((f'link{i}', f'joint{i}') for i in range(1,6)),
            ('ee_base','joint6'), ('ee_link11','ee_link11_joint'), ('ee_link12','ee_link12_joint')))

    @property
    def joint_names(self):
        return tuple(j for _,j in self.joints)

    @property
    def arm_actuators(self):
        return tuple(f'{self.prefix}joint{i}_position' for i in range(1,7))

    @property
    def gripper_actuator(self):
        return f'{self.prefix}ee_fingers_position'

    @property
    def tool_site(self):
        return f'{self.prefix}sarm_ee'

    @property
    def finger_bodies(self):
        return tuple(f'{self.prefix}ee_finger{i}' for i in (1,2))

    @property
    def camera(self):
        return f'{self.prefix}sarm_wrist_cam'

    @property
    def wrench_sensor(self):
        from simulation.wrench_sensors import WrenchSensor
        p=self.prefix
        return WrenchSensor(p+'wrist_ft',p+'wrist_ft_site',p+'wrist_ft_force',p+'wrist_ft_torque',p+'ee_base',p+'wrist_ft_tool')

    def is_arm_body(self, name):
        if not name.startswith(self.prefix): return False
        local = name[len(self.prefix):]
        return local.startswith(('ee','wrist_ft')) or local in {f'link{i}' for i in range(1,6)}

    def validate(self, model):
        root=ET.parse(model).getroot()
        for tag, names in [('joint', self.joint_names), ('site',(self.tool_site,)),
                           ('camera',(self.camera,)), ('body', self.finger_bodies)]:
            for name in names:
                if len(root.findall(f'.//{tag}[@name="{name}"]')) != 1:
                    raise ValueError(f'Missing or duplicate {tag}: {name}')
        for name in (*self.arm_actuators,self.gripper_actuator):
            if root.find(f'./actuator/*[@name="{name}"]') is None:
                raise ValueError(f'Missing actuator {name}')
        for name in (self.wrench_sensor.force_sensor,self.wrench_sensor.torque_sensor):
            if root.find(f'./sensor/*[@name="{name}"]') is None:
                raise ValueError(f'Missing sensor {name}')
        return self


SINGLE_ARM = ArmDescription()
DUAL_ARMS = {'left': ArmDescription('left_'), 'right': ArmDescription('right_')}
