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
        return {}


class Rover2025Gripper(Rover2025GripperBase):
    def format_action(self, action):
        assert len(action) == 1
        return np.array([action[0]])

    @property
    def speed(self):
        return 0.20

    @property
    def dof(self):
        return 1
