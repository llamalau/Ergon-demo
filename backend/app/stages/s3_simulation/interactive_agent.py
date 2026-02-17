"""Interactive agent for playground: receives free-form commands and executes
VLM-generated motion plans as sequences of waypoints.

Architecture: The VLM decides WHAT the robot does (generates a full motion
plan), and this agent simply executes it by interpolating between waypoints.
This enables arbitrary tasks — grasping, waving, pointing, pushing, etc.
"""

import logging
from collections import deque

import numpy as np

from app.stages.s3_simulation.vlm_client import (
    interpret_command,
    MotionPlan,
    MotionStep,
)
from app.stages.s3_simulation.vla_agent import IKController

logger = logging.getLogger(__name__)

# Home position: arm at rest, hand open
_HOME_POS = np.array([0.3, 0.0, 0.6])
_HOME_WRIST = [0.0, -0.3, 0.0]
_OPEN_FINGERS = {"thumb": 0.0, "index": 0.0, "middle": 0.0,
                 "ring": 0.0, "pinky": 0.0}


class InteractiveAgent:
    """Executes VLM-generated motion plans.

    States: idle -> planning -> executing -> idle

    In 'executing', the agent walks through the MotionPlan's steps,
    smoothly interpolating arm position and finger config over each
    step's duration.
    """

    def __init__(self):
        self.phase = "idle"
        self.phase_step = 0
        self.ik: IKController | None = None
        self._command_queue: deque[str] = deque()
        self._last_obs_image: np.ndarray | None = None
        self._object_info: dict = {}

        # Current motion plan execution state
        self._plan: MotionPlan | None = None
        self._current_step_idx: int = 0
        self._step_progress: int = 0
        self._prev_ee: np.ndarray = _HOME_POS.copy()
        self._prev_wrist: list[float] = list(_HOME_WRIST)
        self._prev_fingers: dict[str, float] = dict(_OPEN_FINGERS)

        # Status reporting
        self.phase_changed: bool = False
        self.status_message: str = "Idle — waiting for command"

    def reset(self, model, data, task_config: dict | None = None) -> None:
        import mujoco

        self.phase = "idle"
        self.phase_step = 0
        self.ik = IKController(model)

        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            obj_pos = data.xpos[obj_id].copy()
        except Exception:
            obj_pos = np.array([0.5, 0.0, 0.3])

        self._object_info = {
            "object_position": obj_pos.tolist(),
        }
        self._prev_ee = _HOME_POS.copy()
        self._prev_wrist = list(_HOME_WRIST)
        self._prev_fingers = dict(_OPEN_FINGERS)
        self._set_phase("idle", "Idle — waiting for command")

    def receive_command(self, text: str) -> None:
        self._command_queue.append(text)

    def act(self, model, data, step: int, obs_image=None) -> np.ndarray | None:
        if model.nu == 0:
            return None

        action = np.zeros(model.nu)

        if obs_image is not None:
            self._last_obs_image = obs_image

        # Update object position in scene context
        import mujoco
        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            self._object_info["object_position"] = data.xpos[obj_id].tolist()
        except Exception:
            pass

        if self.phase == "idle":
            self._apply_pose(action, _HOME_POS, _HOME_WRIST, _OPEN_FINGERS)

            if self._command_queue:
                cmd = self._command_queue.popleft()
                self._current_command = cmd
                self._set_phase("planning", f"Planning: {cmd}")

        elif self.phase == "planning":
            # Hold current pose while VLM plans
            self._apply_pose(action, self._prev_ee, self._prev_wrist, self._prev_fingers)
            self._plan = self._call_vlm(self._current_command, obs_image)
            self._current_step_idx = 0
            self._step_progress = 0
            logger.info("Motion plan: %s (%d steps)",
                        self._plan.description, len(self._plan.steps))
            if self._plan.steps:
                reasoning_prefix = ""
                if self._plan.reasoning:
                    reasoning_prefix = f"VLM: {self._plan.reasoning}\n"
                step_desc = self._plan.steps[0].description
                self._set_phase(
                    "executing",
                    f"{reasoning_prefix}Step 1/{len(self._plan.steps)}: {step_desc}",
                )
            else:
                self._set_phase("idle", "Idle — waiting for command")

        elif self.phase == "executing":
            current_step = self._plan.steps[self._current_step_idx]
            t = min(self._step_progress / max(current_step.duration_steps, 1), 1.0)

            # Interpolate from previous pose to current step's target
            ee = self._prev_ee * (1 - t) + np.array(current_step.ee_target) * t
            wrist = [
                self._prev_wrist[i] * (1 - t) + current_step.wrist_orientation[i] * t
                for i in range(3)
            ]
            fingers = {
                f: self._prev_fingers.get(f, 0.0) * (1 - t)
                   + current_step.finger_config.get(f, 0.0) * t
                for f in ["thumb", "index", "middle", "ring", "pinky"]
            }

            self._apply_pose(action, ee, wrist, fingers)
            self._step_progress += 1

            # Advance to next step when duration is reached
            if self._step_progress >= current_step.duration_steps:
                # Snapshot the end pose as start of next interpolation
                self._prev_ee = np.array(current_step.ee_target)
                self._prev_wrist = list(current_step.wrist_orientation)
                self._prev_fingers = dict(current_step.finger_config)
                self._current_step_idx += 1
                self._step_progress = 0

                if self._current_step_idx < len(self._plan.steps):
                    next_step = self._plan.steps[self._current_step_idx]
                    n = len(self._plan.steps)
                    self._set_phase(
                        "executing",
                        f"Step {self._current_step_idx + 1}/{n}: {next_step.description}",
                    )
                else:
                    self._set_phase("idle", "Idle — waiting for command")

        self.phase_step += 1
        return action

    def is_done(self, model, data, step: int) -> bool:
        return False

    # ------------------------------------------------------------------

    def _apply_pose(self, action: np.ndarray, ee: np.ndarray,
                    wrist: list[float], fingers: dict[str, float]) -> None:
        arm_joints = self.ik.compute_arm_joints(np.asarray(ee), wrist)
        finger_joints = self.ik.compute_finger_joints(fingers, "medium")
        self.ik.apply(action, arm_joints)
        self.ik.apply(action, finger_joints)

    def _set_phase(self, phase: str, message: str) -> None:
        self.phase = phase
        self.phase_step = 0
        self.phase_changed = True
        self.status_message = message
        logger.info("Interactive agent → %s: %s", phase, message)

    def _call_vlm(self, command: str, obs_image=None) -> MotionPlan:
        img = obs_image if obs_image is not None else self._last_obs_image
        if img is not None:
            try:
                robot_state = {
                    "ee_position": self._prev_ee.tolist(),
                    "wrist_orientation": list(self._prev_wrist),
                    "finger_state": dict(self._prev_fingers),
                }
                return interpret_command(
                    image=img,
                    command=command,
                    object_info=self._object_info,
                    robot_state=robot_state,
                )
            except Exception as e:
                logger.warning("VLM failed: %s; using fallback", e)

        return MotionPlan(
            description=f"Fallback for: {command}",
            steps=[
                MotionStep(description="Reach forward",
                           ee_target=[0.5, 0.0, 0.4], duration_steps=120),
                MotionStep(description="Return home",
                           ee_target=[0.3, 0.0, 0.6], duration_steps=120),
            ],
        )
