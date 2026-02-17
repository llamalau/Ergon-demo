"""Heuristic scripted grasping agent for the Shadow Hand + 7-DOF arm."""

import numpy as np

# Arm actuator names in order
ARM_ACTUATORS = [
    "act_shoulder_pan",
    "act_shoulder_lift",
    "act_shoulder_roll",
    "act_elbow",
    "act_wrist_roll",
    "act_wrist_pitch",
    "act_wrist_yaw",
]

# Finger joint names grouped by finger
FINGER_GROUPS = {
    "thumb": ["act_thumb_rotation", "act_thumb_abduction", "act_thumb_mcp", "act_thumb_pip"],
    "index": ["act_index_abduction", "act_index_mcp", "act_index_pip", "act_index_dip"],
    "middle": ["act_middle_abduction", "act_middle_mcp", "act_middle_pip", "act_middle_dip"],
    "ring": ["act_ring_abduction", "act_ring_mcp", "act_ring_pip", "act_ring_dip"],
    "pinky": ["act_pinky_abduction", "act_pinky_mcp", "act_pinky_pip", "act_pinky_dip"],
}


class ScriptedAgent:
    """A deterministic scripted agent that performs manipulation using the
    Shadow Hand on a 7-DOF arm.

    Uses a phased approach: position arm → open hand → descend → grasp → lift →
    optionally reorient/transfer → release.
    """

    def __init__(self):
        self.phase = "approach"
        self.target_pos = None
        self.lift_height = 0.5
        self.phase_step = 0
        self.task_type = "grasp"
        self._actuator_map = {}  # name → index
        self._num_actuators = 0

    def reset(self, model, data, task_config: dict):
        self.phase = "approach"
        self.phase_step = 0
        self.task_type = task_config.get("task_type", "grasp")

        self._num_actuators = model.nu
        self._build_actuator_map(model)

        # Find the object's initial position
        import mujoco
        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            self.target_pos = data.xpos[obj_id].copy()
        except Exception:
            self.target_pos = np.array([0.5, 0.0, 0.3])

    def _build_actuator_map(self, model):
        """Map actuator names to their indices."""
        import mujoco
        self._actuator_map = {}
        for i in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self._actuator_map[name] = i

    def _set_actuator(self, action, name, value):
        """Set a named actuator's control value."""
        idx = self._actuator_map.get(name)
        if idx is not None:
            action[idx] = value

    def _set_arm_target(self, action, target_pos):
        """Use simple IK heuristic to move the end-effector toward target_pos.

        Maps Cartesian target to arm joint angles using geometric heuristics.
        """
        x, y, z = target_pos

        # Shoulder pan: rotate toward x-y target
        shoulder_pan = np.arctan2(y, max(x, 0.01))
        # Shoulder lift: angle based on height and forward reach
        r_horizontal = np.sqrt(x**2 + y**2)
        shoulder_lift = np.arctan2(z - 0.5, r_horizontal) - 0.5
        # Shoulder roll: keep hand oriented down
        shoulder_roll = 0.0
        # Elbow: bend based on distance
        reach = np.sqrt(r_horizontal**2 + (z - 0.5)**2)
        elbow = -np.clip((0.6 - reach) * 3.0, -2.0, 2.0)
        # Wrist: orient palm downward for grasping
        wrist_roll = 0.0
        wrist_pitch = -0.3
        wrist_yaw = 0.0

        arm_values = [shoulder_pan, shoulder_lift, shoulder_roll, elbow,
                      wrist_roll, wrist_pitch, wrist_yaw]
        for act_name, val in zip(ARM_ACTUATORS, arm_values):
            self._set_actuator(action, act_name, val)

    def _set_fingers_open(self, action):
        """Open all fingers to pre-grasp configuration."""
        for finger, joints in FINGER_GROUPS.items():
            for jname in joints:
                if "abduction" in jname or "rotation" in jname:
                    self._set_actuator(action, jname, 0.0)
                else:
                    self._set_actuator(action, jname, 0.0)

    def _set_fingers_grasp(self, action, firmness=1.0):
        """Close fingers around object.

        firmness: 0.0 (open) to 1.0 (fully closed)
        """
        # Thumb opposes the other fingers
        self._set_actuator(action, "act_thumb_rotation", -0.5 * firmness)
        self._set_actuator(action, "act_thumb_abduction", 0.3 * firmness)
        self._set_actuator(action, "act_thumb_mcp", 1.2 * firmness)
        self._set_actuator(action, "act_thumb_pip", 0.8 * firmness)

        # Index through pinky: curl inward
        for finger in ["index", "middle", "ring", "pinky"]:
            joints = FINGER_GROUPS[finger]
            self._set_actuator(action, joints[0], 0.0)  # abduction neutral
            self._set_actuator(action, joints[1], 1.3 * firmness)   # mcp
            self._set_actuator(action, joints[2], 1.0 * firmness)   # pip
            self._set_actuator(action, joints[3], 0.7 * firmness)   # dip

    def act(self, model, data, step: int, obs_image=None) -> np.ndarray | None:
        """Compute control action based on current phase."""
        if model.nu == 0:
            return None

        action = np.zeros(model.nu)

        import mujoco
        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            obj_pos = data.xpos[obj_id].copy()
        except Exception:
            obj_pos = self.target_pos.copy()

        if self.phase == "approach":
            # Move arm above the object with fingers open
            target = obj_pos.copy()
            target[2] += 0.15  # 15cm above object
            self._set_arm_target(action, target)
            self._set_fingers_open(action)
            if self.phase_step > 150:
                self.phase = "descend"
                self.phase_step = 0

        elif self.phase == "descend":
            # Lower arm toward object
            target = obj_pos.copy()
            target[2] += 0.02  # Just above object
            self._set_arm_target(action, target)
            self._set_fingers_open(action)
            if self.phase_step > 100:
                self.phase = "pre_grasp"
                self.phase_step = 0

        elif self.phase == "pre_grasp":
            # Spread fingers wide for envelope grasp
            target = obj_pos.copy()
            target[2] += 0.02
            self._set_arm_target(action, target)
            self._set_fingers_grasp(action, firmness=0.2)
            if self.phase_step > 30:
                self.phase = "grasp"
                self.phase_step = 0

        elif self.phase == "grasp":
            # Close fingers around object
            target = obj_pos.copy()
            self._set_arm_target(action, target)
            progress = min(self.phase_step / 60.0, 1.0)
            self._set_fingers_grasp(action, firmness=progress)
            if self.phase_step > 80:
                self.phase = "lift"
                self.phase_step = 0

        elif self.phase == "lift":
            # Raise arm with object grasped
            target = obj_pos.copy()
            lift_progress = min(self.phase_step / 150.0, 1.0)
            target[2] = obj_pos[2] + (self.lift_height - obj_pos[2]) * lift_progress
            self._set_arm_target(action, target)
            self._set_fingers_grasp(action, firmness=1.0)
            if self.phase_step > 200:
                if self.task_type == "reorient":
                    self.phase = "reorient"
                    self.phase_step = 0
                elif self.task_type == "transfer":
                    self.phase = "transfer"
                    self.phase_step = 0
                else:
                    self.phase = "hold"
                    self.phase_step = 0

        elif self.phase == "reorient":
            # Rotate wrist while maintaining grasp
            target = obj_pos.copy()
            target[2] = self.lift_height
            self._set_arm_target(action, target)
            # Apply wrist rotation
            angle = (self.phase_step / 100.0) * np.pi * 0.5
            self._set_actuator(action, "act_wrist_roll", angle)
            self._set_fingers_grasp(action, firmness=1.0)
            if self.phase_step > 150:
                self.phase = "hold"
                self.phase_step = 0

        elif self.phase == "transfer":
            # Move arm horizontally while holding object
            target = self.target_pos.copy()
            transfer_progress = min(self.phase_step / 150.0, 1.0)
            target[0] += 0.3 * transfer_progress
            target[2] = self.lift_height
            self._set_arm_target(action, target)
            self._set_fingers_grasp(action, firmness=1.0)
            if self.phase_step > 200:
                self.phase = "release"
                self.phase_step = 0

        elif self.phase == "release":
            # Open fingers to release object
            release_progress = min(self.phase_step / 40.0, 1.0)
            self._set_fingers_grasp(action, firmness=1.0 - release_progress)
            if self.phase_step > 60:
                self.phase = "done"

        elif self.phase == "hold":
            # Hold position briefly then done
            target = obj_pos.copy()
            target[2] = self.lift_height
            self._set_arm_target(action, target)
            self._set_fingers_grasp(action, firmness=1.0)
            if self.phase_step > 100:
                self.phase = "done"

        self.phase_step += 1
        return action

    def is_done(self, model, data, step: int) -> bool:
        return self.phase == "done" or step > 1500

    def evaluate_success(self, model, data, task_config: dict) -> dict:
        """Evaluate success criteria for the task."""
        import mujoco

        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            obj_pos = data.xpos[obj_id]
        except Exception:
            obj_pos = np.array([0, 0, 0])

        task_type = task_config.get("task_type", "grasp")
        result = {"task_type": task_type, "final_object_position": obj_pos.tolist()}

        if task_type == "grasp":
            result["lifted"] = bool(obj_pos[2] > 0.15)
            result["success"] = result["lifted"]
        elif task_type == "reorient":
            # Check if object was lifted and is still held
            result["lifted"] = bool(obj_pos[2] > 0.15)
            result["success"] = result["lifted"]
        elif task_type == "transfer":
            result["transferred"] = bool(
                np.linalg.norm(obj_pos[:2] - np.array([0.8, 0.0])) < 0.15
            )
            result["lifted"] = bool(obj_pos[2] > 0.1)
            result["success"] = result["transferred"] or result["lifted"]
        else:
            result["success"] = self.phase == "done"

        return result
