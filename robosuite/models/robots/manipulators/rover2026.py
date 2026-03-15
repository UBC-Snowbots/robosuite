import numpy as np

from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion


class Rover2026(ManipulatorModel):
    """
    Rover2026 is the robot created by UBC Rover for the 2025-2026 competition - Aaron

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right"]

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("robots/rover2026/robot.xml"), idn=idn)
        self.set_joint_attribute(attrib="damping", values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.1)))

    @property
    def default_base(self):
        return "NullMount"

    @property
    def default_gripper(self):
        return {"right": "Rover2026Solenoid"}
        # return {"right": "StickClickerGripper"}

    @property
    def default_controller_config(self):
        return {"right": "default_rover2026"}
    
    @property
    def init_qpos(self):
        # Matches RViz/MoveIt initial_positions.yaml (dev_arm_moveit_config_v3)
        return np.array([
            -1.57,    # shoulder_joint
            -1.57,   # link_1_joint
            0.9,    # link1_link2
            0.0,    # a4_rotation
            1.87,    # a5_rotation
            0.0,    # a6_rotation
        ])
    
    """
    - -2.3447904899721523
    - -1.3109592432033523
    - 0.4222493749261091
    - -0.01067216530286909
    - 1.7332384266110858
    - 0.7774125116998901
    - 0.04076263081598325
"""

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.5))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"

    @property
    def is_mobile(self):
        return False  # Unless your rover base is currently modeled as a mobile base in Mujoco