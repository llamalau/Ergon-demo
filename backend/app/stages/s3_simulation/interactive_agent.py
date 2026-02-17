"""Interactive agent for playground: receives free-form commands and executes
them through a state-machine driven manipulation pipeline."""

import logging
from collections import deque

import numpy as np

from app.stages.s3_simulation.vlm_client import (
    interpret_command,
    ManipulationPlan,
)
from app.stages.s3_simulation.vla_agent import IKController
from app.stages.s3_simulation.scripted_agent import FINGER_GROUPS  # noqa: F401

logger = logging.getLogger(__name__)

# Home position: arm roughly vertical, hand open
_HOME_POS = np.array([0.3, 0.0, 0.6])
_HOME_WRIST = [0.0, -0.3, 0.0]


class InteractiveAgent:
    """State-machine agent that receives natural language commands and
    translates them into arm+hand actions via the VLM.

    Phases: idle -> planning -> approach -> descend -> grasp -> lift -> hold -> idle
    """

    PHASES = ["idle", "planning", "approach", "descend", "grasp", "lift", "hold"]

    def __init__(self):
        self.phase = "idle"
        self.phase_step = 0
        self.plan: ManipulationPlan | None = None
        self.ik: IKController | None = None
        self._command_queue: deque[str] = deque()
        self._initial_obj_pos: np.ndarray = np.array([0.5, 0.0, 0.3])
        self._last_obs_image: np.ndarray | None = None
        self._object_info: dict = {}

        # Status reporting
        self.phase_changed: bool = False
        self.status_message: str = "Idle — waiting for command"

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def reset(self, model, data, task_config: dict | None = None) -> None:
        import mujoco

        self.phase = "idle"
        self.phase_step = 0
        self.ik = IKController(model)

        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            self._initial_obj_pos = data.xpos[obj_id].copy()
        except Exception:
            self._initial_obj_pos = np.array([0.5, 0.0, 0.3])

        self._object_info = {
            "initial_position": self._initial_obj_pos.tolist(),
        }
        self._set_phase("idle", "Idle — waiting for command")

    def receive_command(self, text: str) -> None:
        """Queue a natural language command for execution."""
        self._command_queue.append(text)

    def act(self, model, data, step: int, obs_image=None) -> np.ndarray | None:
        if model.nu == 0:
            return None

        action = np.zeros(model.nu)

        if obs_image is not None:
            self._last_obs_image = obs_image

        import mujoco
        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            obj_pos = data.xpos[obj_id].copy()
        except Exception:
            obj_pos = self._initial_obj_pos.copy()

        # ---- Phase logic ----

        if self.phase == "idle":
            # Move to home position with open hand
            arm_joints = self.ik.compute_arm_joints(_HOME_POS, _HOME_WRIST)
            finger_joints = self.ik.compute_finger_joints(
                {f: 0.0 for f in ["thumb", "index", "middle", "ring", "pinky"]},
                "gentle",
            )
            self.ik.apply(action, arm_joints)
            self.ik.apply(action, finger_joints)

            # Check for queued commands
            if self._command_queue:
                cmd = self._command_queue.popleft()
                self._set_phase("planning", f"Planning: {cmd}")
                self._current_command = cmd

        elif self.phase == "planning":
            # Call VLM to interpret command
            self.plan = self._interpret(self._current_command, obs_image)
            self._set_phase("approach", f"Approaching target")

        elif self.phase == "approach":
            target = np.array(self.plan.ee_target)
            target[2] = obj_pos[2] + 0.15
            arm_joints = self.ik.compute_arm_joints(target, self.plan.wrist_orientation)
            finger_joints = self.ik.compute_finger_joints(
                {f: 0.0 for f in self.plan.finger_config}, "gentle"
            )
            self.ik.apply(action, arm_joints)
            self.ik.apply(action, finger_joints)

            if self.phase_step > 150:
                self._set_phase("descend", "Descending to object")

        elif self.phase == "descend":
            target = np.array(self.plan.grasp_point)
            target[2] += 0.02
            arm_joints = self.ik.compute_arm_joints(target, self.plan.wrist_orientation)
            finger_joints = self.ik.compute_finger_joints(
                {f: 0.1 for f in self.plan.finger_config}, "gentle"
            )
            self.ik.apply(action, arm_joints)
            self.ik.apply(action, finger_joints)

            if self.phase_step > 100:
                self._set_phase("grasp", "Grasping object")

        elif self.phase == "grasp":
            target = np.array(self.plan.grasp_point)
            arm_joints = self.ik.compute_arm_joints(target, self.plan.wrist_orientation)
            progress = min(self.phase_step / 80.0, 1.0)
            finger_config = {
                f: v * progress for f, v in self.plan.finger_config.items()
            }
            finger_joints = self.ik.compute_finger_joints(
                finger_config, self.plan.force_level
            )
            self.ik.apply(action, arm_joints)
            self.ik.apply(action, finger_joints)

            if self.phase_step > 100:
                self._set_phase("lift", "Lifting object")

        elif self.phase == "lift":
            target = np.array(self.plan.ee_target)
            lift_progress = min(self.phase_step / 150.0, 1.0)
            target[2] = obj_pos[2] + (0.5 - obj_pos[2]) * lift_progress
            arm_joints = self.ik.compute_arm_joints(target, self.plan.wrist_orientation)
            finger_joints = self.ik.compute_finger_joints(
                self.plan.finger_config, self.plan.force_level
            )
            self.ik.apply(action, arm_joints)
            self.ik.apply(action, finger_joints)

            if self.phase_step > 200:
                self._set_phase("hold", "Holding object")

        elif self.phase == "hold":
            if self.plan:
                arm_joints = self.ik.compute_arm_joints(
                    np.array(self.plan.ee_target), self.plan.wrist_orientation
                )
                finger_joints = self.ik.compute_finger_joints(
                    self.plan.finger_config, self.plan.force_level
                )
                self.ik.apply(action, arm_joints)
                self.ik.apply(action, finger_joints)

            if self.phase_step > 100:
                self._set_phase("idle", "Idle — waiting for command")

        self.phase_step += 1
        return action

    def is_done(self, model, data, step: int) -> bool:
        # Never finishes on its own — runs until stopped externally
        return False

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _set_phase(self, phase: str, message: str) -> None:
        self.phase = phase
        self.phase_step = 0
        self.phase_changed = True
        self.status_message = message
        logger.info("Interactive agent phase → %s: %s", phase, message)

    def _interpret(self, command: str, obs_image=None) -> ManipulationPlan:
        img = obs_image if obs_image is not None else self._last_obs_image
        if img is not None:
            try:
                return interpret_command(
                    image=img,
                    command=command,
                    object_info=self._object_info,
                )
            except Exception as e:
                logger.warning("VLM interpret failed: %s; using defaults", e)
        return ManipulationPlan()
