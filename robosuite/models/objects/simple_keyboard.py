"""
Simple Keyboard object for robosuite.

A 60-key (60%) QWERTY keyboard that can be pressed by the stick clicker
gripper.  12 control keys are colour-coded; the remaining keys are dark gray
decorative caps.  Each key is an individual collision geom so contact
detection can identify which key was pressed.
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
    A 60-key (60%) QWERTY keyboard with individually named geoms.

    The keyboard sits on a table surface.  60 keycap geoms are arranged in
    5 rows at 19 mm pitch on a dark base plate.  The 12 control keys
    (WASD, UJ, OP, QE, semicolon, period) are colour-coded; all other keys
    are dark gray.  Collisions with the stick clicker tip can be mapped back
    to a specific control key.

    No free joint — the keyboard is fixed in place once positioned.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/simple_keyboard.xml"),
            name=name,
            joints=None,        # Fixed object — no free joint
            obj_type="all",
            duplicate_collision_geoms=False,
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
