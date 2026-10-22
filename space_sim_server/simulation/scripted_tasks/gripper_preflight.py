"""Closed-loop gripper poses for the private collision-preflight model only."""
import xml.etree.ElementTree as ET
import numpy as np
from simulation.serial_chain_kinematics import SerialChainKinematics


class ParallelogramGripper:
    """Validate the delivered parallel-link geometry before using its angle relation.

    Physics still solves the real passive joints. This merely keeps a read-only
    candidate pose from leaving passive fingers in their old closed configuration.
    """
    def __init__(self, model_path):
        root = ET.parse(model_path).getroot()
        self.chains = []
        for side in (1, 2):
            active_name = f"ee_link1{side}_joint"
            passive_names = (f"ee_link2{side}_joint", f"ee_finger{side}_joint")
            site1, site2 = f"ee_link1{side}_finger{side}_site", f"ee_finger{side}_link1{side}_site"
            connection = root.find(f'./equality/connect[@site1="{site1}"]')
            if connection is None or connection.get("site2") != site2:
                raise ValueError("Unsupported gripper loop geometry for collision preflight")
            active = SerialChainKinematics.from_mjcf(model_path, base_body="wrist_ft_tool", joint_names=(active_name,), tool_site=site1)
            passive = SerialChainKinematics.from_mjcf(model_path, base_body="wrist_ft_tool", joint_names=passive_names, tool_site=site2)
            self.chains.append((passive_names, active, passive))
        # Reject a different linkage instead of silently applying SARM relations.
        for opening in np.linspace(0, 1, 9):
            self.passive_positions([opening, -opening])

    def passive_positions(self, drives):
        if np.shape(drives) != (2,) or not np.isfinite(drives).all():
            raise ValueError("Expected two finite gripper drive angles")
        result = {}
        for drive, (names, active, passive) in zip(drives, self.chains):
            values = [float(drive), -float(drive)]
            if np.linalg.norm(active.forward([drive])[0]-passive.forward(values)[0]) > 1e-6:
                raise ValueError("Gripper loop does not close at this candidate pose")
            result.update(zip(names, values))
        return result
