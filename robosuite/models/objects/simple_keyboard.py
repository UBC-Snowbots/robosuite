"""
Simple Keyboard object for robosuite.

A 4x3 grid of flat keys that can be pressed by the stick clicker gripper.
Each key has a named collision geom so contact detection can identify which
key was pressed.
"""

from robosuite.models.objects import MujocoXMLObject
from robosuite.utils.mjcf_utils import xml_path_completion


# All key names in the keyboard (matching the XML geom names)
KEY_NAMES = [
    "none_0", "yaw_pos", "yaw_neg",
    "z_neg", "x_pos", "z_pos",
    "y_pos", "x_neg", "y_neg",
    "none_9", "open_gripper", "close_gripper",
]


class SimpleKeyboardObject(MujocoXMLObject):
    """
    A flat 4x3 keyboard with individually named key geoms for contact detection.

    The keyboard is meant to sit on a table surface. Each key is a thin box
    with a unique geom name (e.g., "key_x_pos") so that collisions with the
    stick clicker tip can be mapped back to a specific key.

    No free joint — the keyboard is fixed in place once positioned.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/simple_keyboard.xml"),
            name=name,
            joints=None,        # Fixed object — no free joint
            obj_type="all",
            duplicate_collision_geoms=True,
        )

    @property
    def key_geom_names(self):
        """
        Returns a list of all key geom names (with the naming prefix applied).
        Useful for checking contacts against the stick tip.
        """
        return [self.naming_prefix + f"key_{k}" for k in KEY_NAMES]

    @property
    def key_site_names(self):
        """
        Returns a list of all key site names (with the naming prefix applied).
        Sites are located on the top surface of each key.
        """
        return [self.naming_prefix + f"key_{k}_site" for k in KEY_NAMES]

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to default sites, provides entries for each key site.
        """
        dic = super().important_sites
        for k in KEY_NAMES:
            dic[k] = self.naming_prefix + f"key_{k}_site"
        return dic
