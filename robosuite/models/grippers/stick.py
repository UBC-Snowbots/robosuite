import numpy as np

from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.utils.mjcf_utils import xml_path_completion


class StickClickerBase(GripperModel):
    """
    A simple stick/stylus end effector with no moving parts.
    Has a small spherical collision tip for pressing buttons/keys.
    """

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("grippers/stick.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        # No joints, so empty array
        return np.array([])

    @property
    def _important_geoms(self):
        return {
            "left_finger": [],
            "right_finger": [],
            "left_fingerpad": ["stick_tip"],
            "right_fingerpad": ["stick_tip"],
        }


class StickClickerGripper(StickClickerBase):
    """
    Passive stick clicker — no actuators, 0 DOF.
    """

    def format_action(self, action):
        return np.array([])

    @property
    def speed(self):
        return 0.0

    @property
    def dof(self):
        return 0