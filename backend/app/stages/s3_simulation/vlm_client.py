"""VLM API client for scene analysis and manipulation planning."""

import base64
import io
import json
import logging
from dataclasses import dataclass, field

import numpy as np

from app.core.config import settings

logger = logging.getLogger(__name__)

SCENE_ANALYSIS_PROMPT = """You are a robotic manipulation planner. You are controlling a Shadow Hand (5-finger dexterous hand) mounted on a 7-DOF arm.

Analyze the scene image and plan a manipulation action for the following task:
Task: {task_description}

Object information:
{object_info}

Current phase: {phase}

Based on the image, provide a manipulation plan as JSON with these fields:
{{
  "grasp_point": [x, y, z],       // 3D point on the object to grasp (meters, world frame)
  "approach_vector": [x, y, z],    // direction to approach from (unit vector)
  "finger_config": {{
    "thumb": 0.0-1.0,              // 0=open, 1=fully closed
    "index": 0.0-1.0,
    "middle": 0.0-1.0,
    "ring": 0.0-1.0,
    "pinky": 0.0-1.0
  }},
  "force_level": "gentle|medium|firm",
  "ee_target": [x, y, z],         // target end-effector position
  "wrist_orientation": [roll, pitch, yaw],  // desired wrist angles in radians
  "notes": "brief reasoning"
}}

Respond with ONLY the JSON object, no other text."""

SUCCESS_EVAL_PROMPT = """You are evaluating the result of a robotic manipulation task.

Task: {task_description}
Initial object position: {initial_pos}
Final object position: {final_pos}

Look at the final scene image and evaluate:
{{
  "success": true/false,
  "confidence": 0.0-1.0,
  "reasoning": "brief explanation",
  "grasp_quality": "poor|fair|good|excellent",
  "stability": "unstable|marginal|stable|very_stable"
}}

Respond with ONLY the JSON object."""


@dataclass
class ManipulationPlan:
    """Structured output from VLM scene analysis."""
    grasp_point: list[float] = field(default_factory=lambda: [0.5, 0.0, 0.3])
    approach_vector: list[float] = field(default_factory=lambda: [0.0, 0.0, -1.0])
    finger_config: dict[str, float] = field(
        default_factory=lambda: {"thumb": 0.5, "index": 0.5, "middle": 0.5,
                                  "ring": 0.5, "pinky": 0.5}
    )
    force_level: str = "medium"
    ee_target: list[float] = field(default_factory=lambda: [0.5, 0.0, 0.4])
    wrist_orientation: list[float] = field(default_factory=lambda: [0.0, -0.3, 0.0])
    notes: str = ""


def _encode_image(image: np.ndarray) -> str:
    """Encode numpy image array to base64 PNG string."""
    from PIL import Image

    if image.dtype != np.uint8:
        image = (np.clip(image, 0, 1) * 255).astype(np.uint8)

    pil_img = Image.fromarray(image)
    buf = io.BytesIO()
    pil_img.save(buf, format="PNG")
    buf.seek(0)
    return base64.standard_b64encode(buf.read()).decode("utf-8")


def _call_anthropic(image_b64: str, prompt: str) -> str:
    """Call Anthropic API with image and text prompt."""
    import anthropic

    client = anthropic.Anthropic(api_key=settings.VLM_API_KEY)
    message = client.messages.create(
        model=settings.VLM_MODEL,
        max_tokens=1024,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/png",
                            "data": image_b64,
                        },
                    },
                    {"type": "text", "text": prompt},
                ],
            }
        ],
    )
    return message.content[0].text


def _call_openai(image_b64: str, prompt: str) -> str:
    """Call OpenAI API with image and text prompt."""
    import openai

    client = openai.OpenAI(api_key=settings.VLM_API_KEY)
    response = client.chat.completions.create(
        model=settings.VLM_MODEL,
        max_tokens=1024,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/png;base64,{image_b64}",
                        },
                    },
                    {"type": "text", "text": prompt},
                ],
            }
        ],
    )
    return response.choices[0].message.content


def _call_vlm(image_b64: str, prompt: str) -> str:
    """Route to the configured VLM provider."""
    provider = settings.VLM_PROVIDER.lower()
    if provider == "anthropic":
        return _call_anthropic(image_b64, prompt)
    elif provider == "openai":
        return _call_openai(image_b64, prompt)
    else:
        raise ValueError(f"Unknown VLM provider: {provider}")


def _parse_json_response(text: str) -> dict:
    """Extract JSON from VLM response, handling markdown code fences."""
    text = text.strip()
    if text.startswith("```"):
        # Strip code fences
        lines = text.split("\n")
        lines = [l for l in lines if not l.strip().startswith("```")]
        text = "\n".join(lines)
    return json.loads(text)


def analyze_scene(
    image: np.ndarray,
    task_description: str,
    object_info: dict,
    phase: str = "initial",
) -> ManipulationPlan:
    """Analyze a scene image and return a manipulation plan.

    Args:
        image: RGB image from camera (H, W, 3) uint8
        task_description: e.g. "grasp the object and lift it 20cm"
        object_info: dict with mass, dimensions, material, etc.
        phase: current manipulation phase for context

    Returns:
        ManipulationPlan with grasp strategy
    """
    image_b64 = _encode_image(image)
    prompt = SCENE_ANALYSIS_PROMPT.format(
        task_description=task_description,
        object_info=json.dumps(object_info, indent=2),
        phase=phase,
    )

    try:
        response_text = _call_vlm(image_b64, prompt)
        data = _parse_json_response(response_text)
        return ManipulationPlan(
            grasp_point=data.get("grasp_point", [0.5, 0.0, 0.3]),
            approach_vector=data.get("approach_vector", [0.0, 0.0, -1.0]),
            finger_config=data.get("finger_config",
                                    {"thumb": 0.5, "index": 0.5, "middle": 0.5,
                                     "ring": 0.5, "pinky": 0.5}),
            force_level=data.get("force_level", "medium"),
            ee_target=data.get("ee_target", [0.5, 0.0, 0.4]),
            wrist_orientation=data.get("wrist_orientation", [0.0, -0.3, 0.0]),
            notes=data.get("notes", ""),
        )
    except Exception as e:
        logger.warning("VLM scene analysis failed: %s; using default plan", e)
        return ManipulationPlan()


def evaluate_success_vlm(
    image: np.ndarray,
    task_description: str,
    initial_pos: list[float],
    final_pos: list[float],
) -> dict:
    """Use VLM to evaluate manipulation success from final scene image."""
    image_b64 = _encode_image(image)
    prompt = SUCCESS_EVAL_PROMPT.format(
        task_description=task_description,
        initial_pos=initial_pos,
        final_pos=final_pos,
    )

    try:
        response_text = _call_vlm(image_b64, prompt)
        return _parse_json_response(response_text)
    except Exception as e:
        logger.warning("VLM success evaluation failed: %s", e)
        return {"success": False, "confidence": 0.0, "reasoning": f"VLM error: {e}"}
