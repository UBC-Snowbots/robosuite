from collections import OrderedDict

import numpy as np

from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import TableArena
from robosuite.models.objects.simple_keyboard import SimpleKeyboardObject, KEY_NAMES
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor


class Type(ManipulationEnv):
    """
    Typing task: a simple keyboard is placed on the table and the robot must
    press individual keys with the stick clicker end-effector.

    The environment exposes per-key contact information so that downstream code
    can detect which key the stick tip is currently pressing.

    Args:
        robots (str or list of str): Robot specification (must be single-arm).
        table_full_size (3-tuple): Table x, y, z dimensions.
        table_friction (3-tuple): MuJoCo friction parameters for the table.
        keyboard_pos_offset (3-tuple): XYZ offset of the keyboard center
            relative to the table center (on the table surface).
        use_object_obs (bool): If True, include keyboard state in observations.
        reward_scale (None or float): Scale factor for the reward.
        **kwargs: Additional keyword arguments passed to ManipulationEnv.
    """

    def __init__(
        self,
        robots,
        env_configuration="default",
        controller_configs=None,
        gripper_types="default",
        base_types="default",
        initialization_noise="default",
        table_full_size=(0.8, 0.8, 0.05),
        table_friction=(1.0, 5e-3, 1e-4),
        keyboard_pos_offset=(0.0, 0.1, 0.0),
        use_camera_obs=True,
        use_object_obs=True,
        reward_scale=1.0,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera="frontview",
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=20,
        lite_physics=True,
        horizon=1000,
        ignore_done=False,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
        camera_segmentations=None,
        renderer="mjviewer",
        renderer_config=None,
        seed=None,
    ):
        # Table settings
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.table_offset = np.array((0, 0, 0.8))

        # Keyboard placement offset on the table surface
        self.keyboard_pos_offset = np.array(keyboard_pos_offset)

        # Reward
        self.reward_scale = reward_scale

        # Observation flags
        self.use_object_obs = use_object_obs

        # Runtime state — tracks which keys are currently pressed
        self.pressed_keys = set()

        super().__init__(
            robots=robots,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            base_types=base_types,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            has_renderer=has_renderer,
            has_offscreen_renderer=has_offscreen_renderer,
            render_camera=render_camera,
            render_collision_mesh=render_collision_mesh,
            render_visual_mesh=render_visual_mesh,
            render_gpu_device_id=render_gpu_device_id,
            control_freq=control_freq,
            lite_physics=lite_physics,
            horizon=horizon,
            ignore_done=ignore_done,
            hard_reset=hard_reset,
            camera_names=camera_names,
            camera_heights=camera_heights,
            camera_widths=camera_widths,
            camera_depths=camera_depths,
            camera_segmentations=camera_segmentations,
            renderer=renderer,
            renderer_config=renderer_config,
            seed=seed,
        )

    def reward(self, action=None):
        """
        Reward based on key presses. Currently returns 1.0 for any key press.
        Override or extend for task-specific reward shaping.
        """
        reward = 0.0
        if len(self.pressed_keys) > 0:
            reward = 1.0
        if self.reward_scale is not None:
            reward *= self.reward_scale
        return reward

    def _load_model(self):
        """
        Loads the arena, robot, and keyboard into the simulation.
        """
        super()._load_model()

        # Position robot relative to table
        xpos = self.robots[0].robot_model.base_xpos_offset["table"](self.table_full_size[0])
        self.robots[0].robot_model.set_base_xpos(xpos)

        # Create table arena
        mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_friction=self.table_friction,
            table_offset=self.table_offset,
        )
        mujoco_arena.set_origin([0, 0, 0])

        # Create the keyboard object (fixed — no joints)
        self.keyboard = SimpleKeyboardObject(name="keyboard")

        # Build the task
        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=self.keyboard,
        )

    def _setup_references(self):
        """
        Set up references to the keyboard body and key geom IDs.
        """
        super()._setup_references()

        # Keyboard body
        self.keyboard_body_id = self.sim.model.body_name2id(self.keyboard.root_body)

        # Map each key name to its geom ID for fast contact lookup
        self.key_geom_ids = {}
        for key_name in KEY_NAMES:
            full_name = self.keyboard.naming_prefix + f"key_{key_name}"
            try:
                self.key_geom_ids[key_name] = self.sim.model.geom_name2id(full_name)
            except Exception:
                pass  # Skip keys that couldn't be resolved

        # Find the stick tip geom ID (from the gripper)
        self.stick_tip_geom_id = None
        for i in range(self.sim.model.ngeom):
            name = self.sim.model.geom_id2name(i)
            if name and "stick_tip" in name:
                self.stick_tip_geom_id = i
                break

    def _setup_observables(self):
        """
        Set up observables including keyboard-related observations.
        """
        observables = super()._setup_observables()

        if self.use_object_obs:
            modality = "object"

            @sensor(modality=modality)
            def keyboard_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.keyboard_body_id])

            @sensor(modality=modality)
            def pressed_keys_obs(obs_cache):
                """Binary vector: 1.0 if key is pressed, 0.0 otherwise."""
                vec = np.zeros(len(KEY_NAMES))
                for i, key_name in enumerate(KEY_NAMES):
                    if key_name in self.pressed_keys:
                        vec[i] = 1.0
                return vec

            sensors = [keyboard_pos, pressed_keys_obs]

            # EEF to keyboard distance
            arm_prefixes = self._get_arm_prefixes(self.robots[0], include_robot_name=False)
            full_prefixes = self._get_arm_prefixes(self.robots[0])
            sensors += [
                self._get_obj_eef_sensor(full_pf, "keyboard_pos", f"{arm_pf}gripper_to_keyboard_pos", modality)
                for arm_pf, full_pf in zip(arm_prefixes, full_prefixes)
            ]

            names = [s.__name__ for s in sensors]
            for name, s in zip(names, sensors):
                observables[name] = Observable(
                    name=name,
                    sensor=s,
                    sampling_rate=self.control_freq,
                )

        return observables

    def _reset_internal(self):
        """
        Reset the keyboard position on the table.
        """
        super()._reset_internal()
        self.pressed_keys = set()

        # Place the keyboard at a fixed position on the table
        keyboard_pos = self.table_offset + self.keyboard_pos_offset
        # Nudge it up to sit on the table surface (half table thickness + small offset)
        keyboard_pos[2] += self.table_full_size[2] / 2.0 + 0.005

        # Set keyboard body position directly
        keyboard_body_id = self.sim.model.body_name2id(self.keyboard.root_body)
        self.sim.model.body_pos[keyboard_body_id] = keyboard_pos

    def _post_action(self, action):
        """
        After each action step, detect which keys are being pressed.
        """
        reward, done, info = super()._post_action(action)

        # Detect key presses via MuJoCo contacts
        self.pressed_keys = set()
        if self.stick_tip_geom_id is not None:
            for i in range(self.sim.data.ncon):
                contact = self.sim.data.contact[i]
                g1, g2 = contact.geom1, contact.geom2

                # Check if stick tip is one of the contact geoms
                if self.stick_tip_geom_id in (g1, g2):
                    other = g2 if g1 == self.stick_tip_geom_id else g1
                    # Check if the other geom is a key
                    for key_name, geom_id in self.key_geom_ids.items():
                        if other == geom_id:
                            self.pressed_keys.add(key_name)

        info["pressed_keys"] = list(self.pressed_keys)
        return reward, done, info

    def get_pressed_keys(self):
        """
        Public API to query which keys are currently pressed.

        Returns:
            set: Set of key name strings currently in contact with the stick tip.
        """
        return self.pressed_keys

    def _check_success(self):
        """
        No specific success condition — override for your task.
        """
        return False

    def visualize(self, vis_settings):
        """
        Visualize gripper proximity to the keyboard.
        """
        super().visualize(vis_settings=vis_settings)
        if vis_settings.get("grippers", False):
            self._visualize_gripper_to_target(
                gripper=self.robots[0].gripper, target=self.keyboard
            )
