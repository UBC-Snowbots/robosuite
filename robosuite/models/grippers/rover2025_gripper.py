import numpy as np

from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.utils.mjcf_utils import xml_path_completion


class Rover2025GripperBase(GripperModel):

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("grippers/rover2025_gripper.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.0, 0.0])

    @property
    def _important_geoms(self):
        # Return raw XML geom names (not name-adjusted with prefix).
        # Match the geoms we defined in rover2025_gripper.xml.
        return {
            "left_finger": ["gripper1"],
            "right_finger": ["gripper2"],
            "left_fingerpad": ["left_fingerpad"],
            "right_fingerpad": ["right_fingerpad"],
        }


class Rover2025Gripper(Rover2025GripperBase):
    def format_action(self, action):
        assert len(action) == 1
        return np.array([action[0], action[0]])

    @property
    def speed(self):
        return 0.20

    @property
    def dof(self):
        return 1

    # Testing
    @property
    def grasp_qpos(self):
        # Just return the target for the 'virtual' single DOF
        return {
            1: np.array([0.8]), 
            -1: np.array([-0.8])
        }