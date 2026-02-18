"""High-level task planner using VLM (Claude API).

Decomposes natural language tasks into sub-goals for the low-level controller,
and verifies success via visual comparison.
"""

import json
import logging
from dataclasses import dataclass, field

import numpy as np

from app.stages.s3_simulation.vlm_client import _call_vlm, _encode_image, _parse_json_response

logger = logging.getLogger(__name__)

TASK_DECOMPOSITION_PROMPT = """You are a high-level task planner for a Unitree G1 humanoid robot \
performing manipulation tasks in a MuJoCo simulation.

The G1 is a full-body humanoid (29 DOF with dexterous hands) standing in front of a workspace.
It has two arms with 3-finger hands (thumb, index, middle per hand).

Given the task description and the current scene image, decompose the task into
a sequence of sub-goals. Each sub-goal should be a clear, actionable instruction
that a low-level robot controller can execute autonomously.

Task: {task_description}

Scene context: {scene_context}

Return JSON:
{{
  "reasoning": "Brief analysis of the scene and your strategy",
  "sub_goals": [
    {{
      "description": "What the robot should do (natural language instruction for controller)",
      "success_criteria": "How to visually verify this sub-goal succeeded",
      "expected_duration_steps": 500
    }}
  ]
}}

Guidelines:
- Break complex tasks into 2-6 sub-goals
- Each sub-goal should be self-contained enough for a robot controller to execute
- Include approach, grasp, manipulation, and verification phases as needed
- Be specific about which hand to use and what motion to perform
- Duration: 200=quick action, 500=normal, 1000=careful/complex

Respond with ONLY the JSON object."""

SUCCESS_VERIFICATION_PROMPT = """You are verifying whether a humanoid robot successfully \
completed a manipulation sub-goal.

Sub-goal: {sub_goal_description}
Success criteria: {success_criteria}
Overall task: {task_description}

Image 1 (BEFORE): The scene before the sub-goal was attempted.
Image 2 (AFTER): The scene after the attempt.

Return JSON:
{{
  "success": true or false,
  "confidence": 0.0 to 1.0,
  "reasoning": "What you observe changed or didn't change",
  "should_retry": true or false,
  "retry_hint": "If retrying, what should the robot do differently"
}}

Respond with ONLY the JSON object."""

TASK_COMPLETION_PROMPT = """You are evaluating whether a humanoid robot successfully \
completed an overall manipulation task.

Task: {task_description}

Image 1 (INITIAL): The scene before any manipulation.
Image 2 (FINAL): The scene after all attempts.

Return JSON:
{{
  "success": true or false,
  "confidence": 0.0 to 1.0,
  "reasoning": "Brief explanation of the outcome"
}}

Respond with ONLY the JSON object."""


@dataclass
class SubGoal:
    description: str
    success_criteria: str
    expected_duration_steps: int = 500
    completed: bool = False
    success: bool = False
    attempts: int = 0


@dataclass
class TaskPlan:
    task_description: str
    reasoning: str
    sub_goals: list[SubGoal] = field(default_factory=list)
    current_sub_goal_index: int = 0


class HighLevelPlanner:
    """VLM-based task decomposition and verification."""

    def decompose_task(self, task_description: str, scene_image: np.ndarray,
                       scene_context: str = "") -> TaskPlan:
        """Decompose a task into sub-goals using the VLM."""
        image_b64 = _encode_image(scene_image)
        prompt = TASK_DECOMPOSITION_PROMPT.format(
            task_description=task_description,
            scene_context=scene_context or "A humanoid robot standing in front of a table with objects.",
        )

        try:
            response = _call_vlm(image_b64, prompt)
            data = _parse_json_response(response)

            sub_goals = []
            for sg in data.get("sub_goals", []):
                sub_goals.append(SubGoal(
                    description=sg.get("description", "Reach toward the object"),
                    success_criteria=sg.get("success_criteria", "Robot hand is near object"),
                    expected_duration_steps=int(sg.get("expected_duration_steps", 500)),
                ))

            if not sub_goals:
                raise ValueError("VLM returned no sub-goals")

            return TaskPlan(
                task_description=task_description,
                reasoning=data.get("reasoning", ""),
                sub_goals=sub_goals,
            )
        except Exception as e:
            logger.warning("Task decomposition failed: %s; using fallback plan", e)
            return TaskPlan(
                task_description=task_description,
                reasoning=f"Fallback plan (VLM error: {e})",
                sub_goals=[
                    SubGoal(
                        description=f"Attempt to: {task_description}",
                        success_criteria="Task appears completed",
                        expected_duration_steps=1000,
                    ),
                ],
            )

    def verify_sub_goal(self, sub_goal: SubGoal, before_image: np.ndarray,
                        after_image: np.ndarray, task_description: str) -> dict:
        """Verify whether a sub-goal was achieved by comparing before/after images."""
        before_b64 = _encode_image(before_image)
        after_b64 = _encode_image(after_image)

        prompt = SUCCESS_VERIFICATION_PROMPT.format(
            sub_goal_description=sub_goal.description,
            success_criteria=sub_goal.success_criteria,
            task_description=task_description,
        )

        try:
            # Send both images - before then after
            response = _call_vlm_two_images(before_b64, after_b64, prompt)
            return _parse_json_response(response)
        except Exception as e:
            logger.warning("Sub-goal verification failed: %s", e)
            return {"success": False, "confidence": 0.0, "reasoning": str(e),
                    "should_retry": True, "retry_hint": ""}

    def verify_task_completion(self, task_description: str,
                               initial_image: np.ndarray,
                               final_image: np.ndarray) -> dict:
        """Verify overall task completion."""
        initial_b64 = _encode_image(initial_image)
        final_b64 = _encode_image(final_image)

        prompt = TASK_COMPLETION_PROMPT.format(task_description=task_description)

        try:
            response = _call_vlm_two_images(initial_b64, final_b64, prompt)
            return _parse_json_response(response)
        except Exception as e:
            logger.warning("Task completion verification failed: %s", e)
            return {"success": False, "confidence": 0.0, "reasoning": str(e)}

    def get_current_sub_goal_prompt(self, plan: TaskPlan) -> str:
        """Get the language prompt for the current sub-goal."""
        if plan.current_sub_goal_index >= len(plan.sub_goals):
            return "Stand still and maintain current position."
        return plan.sub_goals[plan.current_sub_goal_index].description


def _call_vlm_two_images(image1_b64: str, image2_b64: str, prompt: str) -> str:
    """Call VLM with two images (before/after comparison)."""
    from app.core.config import settings

    provider = settings.VLM_PROVIDER.lower()
    if provider == "anthropic":
        return _call_anthropic_two_images(image1_b64, image2_b64, prompt)
    elif provider == "openai":
        return _call_openai_two_images(image1_b64, image2_b64, prompt)
    else:
        raise ValueError(f"Unknown VLM provider: {provider}")


def _call_anthropic_two_images(img1_b64: str, img2_b64: str, prompt: str) -> str:
    import anthropic
    from app.core.config import settings

    client = anthropic.Anthropic(api_key=settings.VLM_API_KEY)
    message = client.messages.create(
        model=settings.VLM_MODEL,
        max_tokens=1024,
        messages=[{
            "role": "user",
            "content": [
                {"type": "text", "text": "Image 1 (BEFORE):"},
                {"type": "image", "source": {"type": "base64", "media_type": "image/png", "data": img1_b64}},
                {"type": "text", "text": "Image 2 (AFTER):"},
                {"type": "image", "source": {"type": "base64", "media_type": "image/png", "data": img2_b64}},
                {"type": "text", "text": prompt},
            ],
        }],
    )
    return message.content[0].text


def _call_openai_two_images(img1_b64: str, img2_b64: str, prompt: str) -> str:
    import openai
    from app.core.config import settings

    client = openai.OpenAI(api_key=settings.VLM_API_KEY)
    response = client.chat.completions.create(
        model=settings.VLM_MODEL,
        max_tokens=1024,
        messages=[{
            "role": "user",
            "content": [
                {"type": "text", "text": "Image 1 (BEFORE):"},
                {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{img1_b64}"}},
                {"type": "text", "text": "Image 2 (AFTER):"},
                {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{img2_b64}"}},
                {"type": "text", "text": prompt},
            ],
        }],
    )
    return response.choices[0].message.content
