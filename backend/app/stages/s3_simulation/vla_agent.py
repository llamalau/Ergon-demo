"""VLM-based Vision-Language-Action agent for dexterous manipulation."""

import logging
from abc import ABC, abstractmethod

import numpy as np

from app.stages.s3_simulation.vlm_client import (
    analyze_scene,
    evaluate_success_vlm,
    ManipulationPlan,
)
from app.stages.s3_simulation.scripted_agent import ARM_ACTUATORS, FINGER_GROUPS

logger = logging.getLogger(__name__)


class VLAAgent(ABC):
    """Abstract base class for Vision-Language-Action model agents."""

    @abstractmethod
    def reset(self, model, data, task_config: dict) -> None:
        ...

    @abstractmethod
    def act(self, model, data, step: int, obs_image=None) -> np.ndarray | None:
        ...

    @abstractmethod
    def is_done(self, model, data, step: int) -> bool:
        ...

    @abstractmethod
    def evaluate_success(self, model, data, task_config: dict) -> dict:
        ...


class IKController:
    """Simple inverse kinematics controller that maps Cartesian targets
    to arm joint angles using geometric heuristics."""

    def __init__(self, model):
        self._model = model
        self._actuator_map = {}
        import mujoco
        for i in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self._actuator_map[name] = i

    def compute_arm_joints(self, target_pos: np.ndarray,
                           wrist_orientation: list[float] | None = None) -> dict[str, float]:
        """Compute arm joint targets for a desired end-effector position."""
        x, y, z = target_pos

        shoulder_pan = np.arctan2(y, max(x, 0.01))
        r_horizontal = np.sqrt(x**2 + y**2)
        shoulder_lift = np.arctan2(z - 0.5, r_horizontal) - 0.5
        shoulder_roll = 0.0
        reach = np.sqrt(r_horizontal**2 + (z - 0.5)**2)
        elbow = -np.clip((0.6 - reach) * 3.0, -2.0, 2.0)

        if wrist_orientation:
            wrist_roll, wrist_pitch, wrist_yaw = wrist_orientation
        else:
            wrist_roll = 0.0
            wrist_pitch = -0.3
            wrist_yaw = 0.0

        return {
            "act_shoulder_pan": shoulder_pan,
            "act_shoulder_lift": shoulder_lift,
            "act_shoulder_roll": shoulder_roll,
            "act_elbow": elbow,
            "act_wrist_roll": wrist_roll,
            "act_wrist_pitch": wrist_pitch,
            "act_wrist_yaw": wrist_yaw,
        }

    def compute_finger_joints(self, finger_config: dict[str, float],
                               force_level: str = "medium") -> dict[str, float]:
        """Compute finger joint targets from VLM finger configuration."""
        force_scale = {"gentle": 0.6, "medium": 0.8, "firm": 1.0}.get(force_level, 0.8)
        joints = {}

        for finger, closedness in finger_config.items():
            if finger not in FINGER_GROUPS:
                continue
            actuator_names = FINGER_GROUPS[finger]
            val = closedness * force_scale

            if finger == "thumb":
                joints[actuator_names[0]] = -0.5 * val    # rotation
                joints[actuator_names[1]] = 0.3 * val      # abduction
                joints[actuator_names[2]] = 1.2 * val      # mcp
                joints[actuator_names[3]] = 0.8 * val      # pip
            else:
                joints[actuator_names[0]] = 0.0            # abduction
                joints[actuator_names[1]] = 1.3 * val      # mcp
                joints[actuator_names[2]] = 1.0 * val      # pip
                joints[actuator_names[3]] = 0.7 * val      # dip

        return joints

    def apply(self, action: np.ndarray, joint_targets: dict[str, float]):
        """Apply joint targets to the action array."""
        for name, value in joint_targets.items():
            idx = self._actuator_map.get(name)
            if idx is not None:
                action[idx] = value


class VLMAgent(VLAAgent):
    """VLM-powered manipulation agent.

    Calls the VLM API at low frequency (phase transitions) for high-level planning,
    and uses a local IK controller at full sim rate for smooth execution.
    """

    # Phases and their transition logic
    PHASE_SEQUENCE = ["plan", "approach", "descend", "pre_grasp",
                      "grasp", "lift", "task_action", "hold", "done"]

    def __init__(self, config: dict | None = None):
        self.config = config or {}
        self.phase = "plan"
        self.phase_step = 0
        self.task_type = "grasp"
        self.plan: ManipulationPlan | None = None
        self.ik: IKController | None = None
        self._initial_obj_pos = None
        self._last_obs_image = None
        self._object_info = {}
        self._num_actuators = 0

    def reset(self, model, data, task_config: dict) -> None:
        self.phase = "plan"
        self.phase_step = 0
        self.task_type = task_config.get("task_type", "grasp")
        self._num_actuators = model.nu
        self.ik = IKController(model)

        # Track object initial position
        import mujoco
        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            self._initial_obj_pos = data.xpos[obj_id].copy()
        except Exception:
            self._initial_obj_pos = np.array([0.5, 0.0, 0.3])

        self._object_info = {
            "initial_position": self._initial_obj_pos.tolist(),
            "task_type": self.task_type,
        }

    def _get_task_description(self) -> str:
        descs = {
            "grasp": "Grasp the object securely and lift it 20cm above its starting position.",
            "reorient": "Grasp the object, lift it, and rotate it 90 degrees.",
            "transfer": "Grasp the object, lift it, and move it 30cm to the right.",
            "use": "Grasp the object and demonstrate a functional use motion.",
        }
        return descs.get(self.task_type, "Grasp and manipulate the object.")

    def _request_plan(self, obs_image, phase: str) -> ManipulationPlan:
        """Call VLM for a new manipulation plan."""
        if obs_image is not None:
            self._last_obs_image = obs_image

        if self._last_obs_image is not None:
            try:
                plan = analyze_scene(
                    image=self._last_obs_image,
                    task_description=self._get_task_description(),
                    object_info=self._object_info,
                    phase=phase,
                )
                logger.info("VLM plan for phase %s: %s", phase, plan.notes)
                return plan
            except Exception as e:
                logger.warning("VLM planning failed for phase %s: %s", phase, e)

        return ManipulationPlan()

    def act(self, model, data, step: int, obs_image=None) -> np.ndarray | None:
        if model.nu == 0:
            return None

        action = np.zeros(model.nu)

        # Store latest observation
        if obs_image is not None:
            self._last_obs_image = obs_image

        import mujoco
        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            obj_pos = data.xpos[obj_id].copy()
        except Exception:
            obj_pos = self._initial_obj_pos.copy()

        # Phase transitions with VLM replanning
        if self.phase == "plan":
            self.plan = self._request_plan(obs_image, "initial")
            self.phase = "approach"
            self.phase_step = 0

        elif self.phase == "approach":
            target = np.array(self.plan.ee_target)
            target[2] = obj_pos[2] + 0.15
            arm_joints = self.ik.compute_arm_joints(
                target, self.plan.wrist_orientation
            )
            finger_joints = self.ik.compute_finger_joints(
                {f: 0.0 for f in self.plan.finger_config}, "gentle"
            )
            self.ik.apply(action, arm_joints)
            self.ik.apply(action, finger_joints)

            if self.phase_step > 150:
                self.plan = self._request_plan(obs_image, "pre_grasp")
                self.phase = "descend"
                self.phase_step = 0

        elif self.phase == "descend":
            target = np.array(self.plan.grasp_point)
            target[2] += 0.02
            arm_joints = self.ik.compute_arm_joints(
                target, self.plan.wrist_orientation
            )
            finger_joints = self.ik.compute_finger_joints(
                {f: 0.1 for f in self.plan.finger_config}, "gentle"
            )
            self.ik.apply(action, arm_joints)
            self.ik.apply(action, finger_joints)

            if self.phase_step > 100:
                self.phase = "grasp"
                self.phase_step = 0

        elif self.phase == "grasp":
            target = np.array(self.plan.grasp_point)
            arm_joints = self.ik.compute_arm_joints(
                target, self.plan.wrist_orientation
            )
            # Gradually close fingers per VLM config
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
                self.plan = self._request_plan(obs_image, "post_grasp")
                self.phase = "lift"
                self.phase_step = 0

        elif self.phase == "lift":
            target = np.array(self.plan.ee_target)
            lift_progress = min(self.phase_step / 150.0, 1.0)
            target[2] = obj_pos[2] + (0.5 - obj_pos[2]) * lift_progress
            arm_joints = self.ik.compute_arm_joints(
                target, self.plan.wrist_orientation
            )
            finger_joints = self.ik.compute_finger_joints(
                self.plan.finger_config, self.plan.force_level
            )
            self.ik.apply(action, arm_joints)
            self.ik.apply(action, finger_joints)

            if self.phase_step > 200:
                if self.task_type in ("reorient", "transfer", "use"):
                    self.plan = self._request_plan(obs_image, self.task_type)
                    self.phase = "task_action"
                    self.phase_step = 0
                else:
                    self.phase = "hold"
                    self.phase_step = 0

        elif self.phase == "task_action":
            # Task-specific actions guided by VLM plan
            target = np.array(self.plan.ee_target)
            if self.task_type == "reorient":
                angle = (self.phase_step / 100.0) * np.pi * 0.5
                wrist = list(self.plan.wrist_orientation)
                wrist[0] += angle
                arm_joints = self.ik.compute_arm_joints(target, wrist)
            elif self.task_type == "transfer":
                progress = min(self.phase_step / 150.0, 1.0)
                target[0] += 0.3 * progress
                arm_joints = self.ik.compute_arm_joints(
                    target, self.plan.wrist_orientation
                )
            else:
                arm_joints = self.ik.compute_arm_joints(
                    target, self.plan.wrist_orientation
                )

            finger_joints = self.ik.compute_finger_joints(
                self.plan.finger_config, self.plan.force_level
            )
            self.ik.apply(action, arm_joints)
            self.ik.apply(action, finger_joints)

            if self.phase_step > 200:
                self.phase = "hold"
                self.phase_step = 0

        elif self.phase == "hold":
            # Maintain grip
            if self.plan:
                arm_joints = self.ik.compute_arm_joints(
                    np.array(self.plan.ee_target),
                    self.plan.wrist_orientation,
                )
                finger_joints = self.ik.compute_finger_joints(
                    self.plan.finger_config, self.plan.force_level
                )
                self.ik.apply(action, arm_joints)
                self.ik.apply(action, finger_joints)

            if self.phase_step > 100:
                self.phase = "done"

        self.phase_step += 1
        return action

    def is_done(self, model, data, step: int) -> bool:
        return self.phase == "done" or step > 1500

    def evaluate_success(self, model, data, task_config: dict) -> dict:
        """Evaluate success, optionally using VLM for richer feedback."""
        import mujoco

        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            obj_pos = data.xpos[obj_id]
        except Exception:
            obj_pos = np.array([0, 0, 0])

        task_type = task_config.get("task_type", "grasp")
        result = {
            "task_type": task_type,
            "final_object_position": obj_pos.tolist(),
            "agent_type": "vla",
        }

        # Basic physics-based success check
        if task_type == "grasp":
            result["lifted"] = bool(obj_pos[2] > 0.15)
            result["success"] = result["lifted"]
        elif task_type == "reorient":
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

        # Optionally use VLM for richer evaluation
        if self._last_obs_image is not None:
            try:
                vlm_eval = evaluate_success_vlm(
                    image=self._last_obs_image,
                    task_description=self._get_task_description(),
                    initial_pos=self._initial_obj_pos.tolist(),
                    final_pos=obj_pos.tolist(),
                )
                result["vlm_evaluation"] = vlm_eval
            except Exception as e:
                logger.warning("VLM evaluation failed: %s", e)

        return result
