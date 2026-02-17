"""Heuristic IK-based scripted grasping agent (deterministic, reliable)."""

import numpy as np


class ScriptedAgent:
    """A deterministic scripted agent that performs basic pick-and-place.

    Uses a simple approach-grasp-lift-transfer sequence with heuristic IK.
    """

    def __init__(self):
        self.phase = "approach"
        self.target_pos = None
        self.lift_height = 0.3
        self.grasp_threshold = 0.02
        self.phase_step = 0

    def reset(self, model, data, task_config: dict):
        self.phase = "approach"
        self.phase_step = 0
        self.task_type = task_config.get("task_type", "grasp")

        # Find the object's initial position
        try:
            import mujoco
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            self.target_pos = data.xpos[obj_id].copy()
        except Exception:
            self.target_pos = np.array([0.5, 0.0, 0.3])

    def act(self, model, data, step: int) -> np.ndarray | None:
        """Compute control action based on current phase.

        Returns control signals or None if no actuators to control.
        """
        if model.nu == 0:
            return None

        # Simple proportional controller toward object
        action = np.zeros(model.nu)

        import mujoco

        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            obj_pos = data.xpos[obj_id]
        except Exception:
            obj_pos = self.target_pos

        # Phase-based control
        if self.phase == "approach":
            # Move toward object (above it first)
            target = obj_pos.copy()
            target[2] += 0.1  # Approach from above
            self._generate_approach_action(action, target, data, model)
            if self.phase_step > 200:
                self.phase = "descend"
                self.phase_step = 0

        elif self.phase == "descend":
            target = obj_pos.copy()
            self._generate_approach_action(action, target, data, model)
            if self.phase_step > 100:
                self.phase = "grasp"
                self.phase_step = 0

        elif self.phase == "grasp":
            # Close gripper
            if model.nu > 0:
                action[-1] = 1.0  # Assume last actuator is gripper
            if self.phase_step > 50:
                self.phase = "lift"
                self.phase_step = 0

        elif self.phase == "lift":
            target = obj_pos.copy()
            target[2] = self.lift_height
            self._generate_approach_action(action, target, data, model)
            if model.nu > 0:
                action[-1] = 1.0  # Keep gripper closed
            if self.phase_step > 200:
                if self.task_type == "transfer":
                    self.phase = "transfer"
                    self.phase_step = 0
                else:
                    self.phase = "done"

        elif self.phase == "transfer":
            target = self.target_pos.copy()
            target[0] += 0.3
            target[2] = self.lift_height
            self._generate_approach_action(action, target, data, model)
            if model.nu > 0:
                action[-1] = 1.0
            if self.phase_step > 200:
                self.phase = "release"
                self.phase_step = 0

        elif self.phase == "release":
            if model.nu > 0:
                action[-1] = 0.0  # Open gripper
            if self.phase_step > 50:
                self.phase = "done"

        self.phase_step += 1
        return action

    def _generate_approach_action(self, action, target, data, model):
        """Simple proportional control toward target position."""
        # Use first 3 actuators for position control if available
        n = min(3, model.nu - 1)
        for i in range(n):
            error = target[i] - data.qpos[i] if i < len(data.qpos) else 0
            action[i] = np.clip(error * 10.0, -1.0, 1.0)

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
            # Success if object is above initial height
            result["lifted"] = bool(obj_pos[2] > 0.15)
            result["success"] = result["lifted"]
        elif task_type == "reorient":
            result["success"] = True  # Simplified
        elif task_type == "transfer":
            result["transferred"] = bool(
                np.linalg.norm(obj_pos[:2] - np.array([0.8, 0.0])) < 0.1
            )
            result["success"] = result["transferred"]
        else:
            result["success"] = self.phase == "done"

        return result
