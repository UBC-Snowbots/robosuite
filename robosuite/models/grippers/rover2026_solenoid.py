import numpy as np

from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.utils.mjcf_utils import xml_path_completion


class Rover2026SolenoidBase(GripperModel):
    """
    Solenoid linear actuator replacing the finger gripper on Rover2026.

    Binary state only: retracted (0.0 m) or extended (0.04 m).
    Contact detection is done via cfrc_ext on the actuator_tip body in
    simulation, and via a5_rotation Moteus torque delta on real hardware.
    """

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("grippers/rover2026_solenoid.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.0])  # retracted

    @property
    def _important_geoms(self):
        return {
            "actuator_tip": ["actuator_tip"],
            # Lift task's _check_grasp expects these keys — map to tip for compatibility
            "left_fingerpad": ["actuator_tip"],
            "right_fingerpad": ["actuator_tip"],
        }


class Rover2026Solenoid(Rover2026SolenoidBase):
    """
    Single-DOF solenoid gripper.

    format_action maps the policy's scalar action ∈ [-1, 1] to a binary
    position command: action > 0 → extend (0.04 m), else → retract (0.0 m).
    The position servo in the XML snaps the joint to the commanded value.
    """

    def format_action(self, action):
        assert len(action) == 1
        # Motor model: >0 energises solenoid (25N extend), ≤0 de-energises (spring retracts)
        return np.array([25.0 if action[0] > 0.0 else 0.0])

    @property
    def speed(self):
        return 1.0  # instantaneous for a solenoid

    @property
    def dof(self):
        return 1
