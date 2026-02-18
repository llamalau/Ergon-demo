"""Two-level agent: VLM planner (high-level) + remote control model (low-level).

The VLM decomposes tasks into sub-goals and verifies success.
The control model (OpenPI/pi0 on SLURM) executes sub-goals by producing
joint-level actions given observations.

Agent interface matches sim_runner expectations:
- reset(model, data, task_config)
- act(model, data, step, obs_image) -> action array
- is_done(model, data, step) -> bool
- evaluate_success(model, data, task_config) -> dict
"""

import logging

import numpy as np

from app.stages.s3_simulation.high_level_planner import HighLevelPlanner, TaskPlan
from app.stages.s3_simulation.low_level_client import LowLevelControllerClient
from app.stages.s3_simulation.camera_manager import CameraManager
from app.stages.s3_simulation.humanoid_loader import STAND_CTRL

logger = logging.getLogger(__name__)


class TwoLevelAgent:
    """Agent combining VLM task planning with model-based motor control."""

    def __init__(self, config: dict | None = None):
        self.config = config or {}
        self.planner = HighLevelPlanner()
        self.controller: LowLevelControllerClient | None = None
        self.camera_manager: CameraManager | None = None
        self.plan: TaskPlan | None = None
        self.phase = "idle"  # idle -> planning -> executing -> done
        self._step_in_subgoal = 0
        self._max_steps_per_subgoal = 1000
        self._replan_interval = 16
        self._initial_image: np.ndarray | None = None
        self._subgoal_start_image: np.ndarray | None = None
        self._controller_available = False

    def reset(self, model, data, task_config: dict) -> None:
        """Initialize agent for a new trial."""
        self.camera_manager = CameraManager(model)
        self._step_in_subgoal = 0
        self.phase = "planning"

        # Try to connect to the low-level controller
        self.controller = LowLevelControllerClient()
        self.controller.reset()
        self._controller_available = self.controller.health_check()
        if not self._controller_available:
            logger.warning("Low-level controller not available; will use standing pose")

        # Initialize humanoid to standing pose
        import mujoco
        stand_ctrl = np.array(STAND_CTRL, dtype=np.float64)
        data.ctrl[:len(stand_ctrl)] = stand_ctrl
        mujoco.mj_step(model, data)

        # Capture initial scene
        self._initial_image = self.camera_manager.get_observation_for_planner(data)
        if self._initial_image is None:
            self._initial_image = np.zeros((256, 256, 3), dtype=np.uint8)

        # Decompose task into sub-goals
        task_description = task_config.get("task_description", "Grasp the object and lift it")
        try:
            self.plan = self.planner.decompose_task(
                task_description=task_description,
                scene_image=self._initial_image,
            )
            logger.info("Task plan: %s sub-goals -- %s",
                        len(self.plan.sub_goals), self.plan.reasoning)
        except Exception as e:
            logger.error("Task decomposition failed: %s", e)
            self.plan = TaskPlan(
                task_description=task_description,
                reasoning=f"Fallback: {e}",
                sub_goals=[],
            )

        self._subgoal_start_image = self._initial_image
        self.phase = "executing" if self.plan.sub_goals else "done"

    def act(self, model, data, step: int, obs_image=None) -> np.ndarray | None:
        """Compute action for current sim step."""
        if self.phase != "executing" or self.plan is None:
            return np.array(STAND_CTRL, dtype=np.float64)

        # Check if current sub-goal exceeded time budget
        if self.plan.current_sub_goal_index >= len(self.plan.sub_goals):
            self.phase = "done"
            return np.array(STAND_CTRL, dtype=np.float64)

        current_sg = self.plan.sub_goals[self.plan.current_sub_goal_index]
        if self._step_in_subgoal >= current_sg.expected_duration_steps:
            self._advance_sub_goal(model, data)
            if self.phase == "done":
                return np.array(STAND_CTRL, dtype=np.float64)

        if not self._controller_available:
            # No controller: maintain standing pose
            return np.array(STAND_CTRL, dtype=np.float64)

        # Get observations for the low-level controller
        obs_images = self.camera_manager.get_observations_for_controller(data)
        proprioception = self._get_proprioception(model, data)
        prompt = self.planner.get_current_sub_goal_prompt(self.plan)

        try:
            action = self.controller.get_next_action(
                images=obs_images,
                proprioception=proprioception,
                prompt=prompt,
                replan_interval=self._replan_interval,
            )
        except Exception as e:
            logger.warning("Controller query failed: %s; holding pose", e)
            action = np.array(STAND_CTRL, dtype=np.float64)

        self._step_in_subgoal += 1
        return action

    def _advance_sub_goal(self, model, data):
        """Verify current sub-goal and advance to next."""
        after_image = self.camera_manager.get_observation_for_planner(data)
        if after_image is None:
            after_image = np.zeros((256, 256, 3), dtype=np.uint8)

        current_sg = self.plan.sub_goals[self.plan.current_sub_goal_index]

        try:
            result = self.planner.verify_sub_goal(
                sub_goal=current_sg,
                before_image=self._subgoal_start_image,
                after_image=after_image,
                task_description=self.plan.task_description,
            )
            current_sg.success = result.get("success", False)
        except Exception as e:
            logger.warning("Sub-goal verification failed: %s", e)
            current_sg.success = False

        current_sg.completed = True
        current_sg.attempts += 1

        logger.info("Sub-goal %d '%s': %s",
                     self.plan.current_sub_goal_index,
                     current_sg.description,
                     "SUCCESS" if current_sg.success else "FAILED")

        # Move to next sub-goal
        self.plan.current_sub_goal_index += 1
        self._step_in_subgoal = 0
        self._subgoal_start_image = after_image

        if self.controller:
            self.controller.reset()

        if self.plan.current_sub_goal_index >= len(self.plan.sub_goals):
            self.phase = "done"

    def _get_proprioception(self, model, data) -> np.ndarray:
        """Extract proprioceptive state: joint positions + velocities."""
        return np.concatenate([data.qpos.copy(), data.qvel.copy()])

    def is_done(self, model, data, step: int) -> bool:
        return self.phase == "done" or step >= 5000

    def evaluate_success(self, model, data, task_config: dict) -> dict:
        """Evaluate overall task success via VLM."""
        final_image = self.camera_manager.get_observation_for_planner(data)
        if final_image is None:
            final_image = np.zeros((256, 256, 3), dtype=np.uint8)

        try:
            vlm_result = self.planner.verify_task_completion(
                task_description=task_config.get("task_description", ""),
                initial_image=self._initial_image,
                final_image=final_image,
            )
        except Exception as e:
            logger.warning("Task verification failed: %s", e)
            vlm_result = {"success": False, "confidence": 0.0, "reasoning": str(e)}

        sub_goal_results = []
        for sg in (self.plan.sub_goals if self.plan else []):
            sub_goal_results.append({
                "description": sg.description,
                "success": sg.success,
                "attempts": sg.attempts,
            })

        return {
            "success": vlm_result.get("success", False),
            "confidence": vlm_result.get("confidence", 0.0),
            "reasoning": vlm_result.get("reasoning", ""),
            "sub_goals": sub_goal_results,
        }
