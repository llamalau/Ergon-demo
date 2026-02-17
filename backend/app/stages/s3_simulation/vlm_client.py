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

COMMAND_INTERPRETATION_PROMPT = """You are controlling a humanoid robot arm with:
- A 7-DOF arm (shoulder pan/lift/roll, elbow, wrist roll/pitch/yaw)
- A 5-finger dexterous hand (thumb, index, middle, ring, pinky)
  Each finger: 0.0 = fully open, 1.0 = fully closed

Camera: You are viewing the scene from a fixed overhead camera at position (1.2, -0.8, 1.0), looking toward the workspace center from an elevated angle (60-degree FOV). The robot arm and table are visible in the image.

Coordinate frame (world frame, meters):
- +X is forward (away from robot base)
- +Y is left
- +Z is up
- The robot base is at approximately (0, 0, 0.5)

Workspace bounds:
- X: 0.15 to 0.75 (forward/back)
- Y: -0.4 to 0.4 (left/right)
- Z: 0.05 to 0.8 (height)
- Wrist orientation: [roll, pitch, yaw] in radians, each -1.6 to 1.6

Robot state:
- Current end-effector position: {robot_ee_pos}
- Current wrist orientation: {robot_wrist}
- Current finger state: {robot_fingers}

Scene context:
{object_info}

IK note: The robot's IK works best within 0.15–0.7m reach from base. Positions near workspace edges may be inaccurate. Prefer moderate, centered positions.

User command: {command}

Plan a sequence of motion steps to execute this command. Each step smoothly
moves the arm to a target pose with a finger configuration over a duration.

Return ONLY a JSON object:
{{
  "reasoning": "1-2 sentences: what you see in the image and your strategy for the command",
  "description": "Brief summary of the overall plan",
  "steps": [
    {{
      "description": "What this step accomplishes",
      "ee_target": [x, y, z],
      "wrist_orientation": [roll, pitch, yaw],
      "finger_config": {{"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}},
      "duration_steps": 100
    }}
  ]
}}

Here are examples of correct output for common commands:

Example 1 — "wave hello":
{{
  "reasoning": "The robot arm is at rest. I'll raise the hand and oscillate the wrist to wave.",
  "description": "Wave hello by raising arm and oscillating wrist",
  "steps": [
    {{"description": "Raise hand up", "ee_target": [0.3, 0.0, 0.75], "wrist_orientation": [0.0, 0.0, 0.0], "finger_config": {{"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}}, "duration_steps": 80}},
    {{"description": "Wave right", "ee_target": [0.3, -0.15, 0.75], "wrist_orientation": [0.8, 0.0, 0.0], "finger_config": {{"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}}, "duration_steps": 60}},
    {{"description": "Wave left", "ee_target": [0.3, 0.15, 0.75], "wrist_orientation": [-0.8, 0.0, 0.0], "finger_config": {{"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}}, "duration_steps": 60}},
    {{"description": "Wave right again", "ee_target": [0.3, -0.15, 0.75], "wrist_orientation": [0.8, 0.0, 0.0], "finger_config": {{"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}}, "duration_steps": 60}},
    {{"description": "Return to rest", "ee_target": [0.3, 0.0, 0.6], "wrist_orientation": [0.0, -0.3, 0.0], "finger_config": {{"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}}, "duration_steps": 80}}
  ]
}}

Example 2 — "pick up the object" (object at [0.5, 0.1, 0.3]):
{{
  "reasoning": "I see an object on the table at roughly [0.5, 0.1, 0.3]. I'll approach from above, descend, grasp, then lift.",
  "description": "Grasp and lift the object",
  "steps": [
    {{"description": "Move above object", "ee_target": [0.5, 0.1, 0.5], "wrist_orientation": [0.0, -1.0, 0.0], "finger_config": {{"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}}, "duration_steps": 100}},
    {{"description": "Descend to object", "ee_target": [0.5, 0.1, 0.32], "wrist_orientation": [0.0, -1.0, 0.0], "finger_config": {{"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}}, "duration_steps": 80}},
    {{"description": "Close fingers to grasp", "ee_target": [0.5, 0.1, 0.32], "wrist_orientation": [0.0, -1.0, 0.0], "finger_config": {{"thumb": 0.8, "index": 0.8, "middle": 0.8, "ring": 0.8, "pinky": 0.8}}, "duration_steps": 60}},
    {{"description": "Lift object", "ee_target": [0.5, 0.1, 0.55], "wrist_orientation": [0.0, -1.0, 0.0], "finger_config": {{"thumb": 0.8, "index": 0.8, "middle": 0.8, "ring": 0.8, "pinky": 0.8}}, "duration_steps": 100}}
  ]
}}

Example 3 — "point at the object" (object at [0.5, 0.1, 0.3]):
{{
  "reasoning": "I need to extend the index finger and aim toward the object while keeping other fingers closed.",
  "description": "Point at the object with index finger",
  "steps": [
    {{"description": "Raise arm toward object", "ee_target": [0.4, 0.08, 0.45], "wrist_orientation": [0.0, -0.5, 0.1], "finger_config": {{"thumb": 0.8, "index": 0.0, "middle": 0.8, "ring": 0.8, "pinky": 0.8}}, "duration_steps": 100}},
    {{"description": "Hold pointing pose", "ee_target": [0.4, 0.08, 0.45], "wrist_orientation": [0.0, -0.5, 0.1], "finger_config": {{"thumb": 0.8, "index": 0.0, "middle": 0.8, "ring": 0.8, "pinky": 0.8}}, "duration_steps": 150}},
    {{"description": "Return to rest", "ee_target": [0.3, 0.0, 0.6], "wrist_orientation": [0.0, -0.3, 0.0], "finger_config": {{"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0, "pinky": 0.0}}, "duration_steps": 100}}
  ]
}}

Guidelines:
- Use 2-8 steps. Each step transitions smoothly from the previous one.
- duration_steps: 50 = fast, 100 = normal, 200 = slow/careful. Range: 20-500.
- Plan relative to the robot's CURRENT position, not always from home.
- For grasping: approach above object → descend to it → close fingers → lift
- For pointing: move arm toward target, extend only index finger (others closed)
- For waving: raise arm high, add oscillating wrist roll steps
- For pushing: approach with closed fist, move through target
- For touching/poking: extend index, move to contact point
- Be creative for unusual commands — map intent to arm+hand motions
- Keep ALL positions within workspace bounds"""

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


@dataclass
class MotionStep:
    """A single waypoint in a motion plan."""
    description: str = ""
    ee_target: list[float] = field(default_factory=lambda: [0.3, 0.0, 0.5])
    wrist_orientation: list[float] = field(default_factory=lambda: [0.0, -0.3, 0.0])
    finger_config: dict[str, float] = field(
        default_factory=lambda: {"thumb": 0.0, "index": 0.0, "middle": 0.0,
                                  "ring": 0.0, "pinky": 0.0}
    )
    duration_steps: int = 100


@dataclass
class MotionPlan:
    """A sequence of motion steps generated by the VLM for any command."""
    description: str = ""
    steps: list[MotionStep] = field(default_factory=list)
    reasoning: str = ""


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


def _clamp(value: float, lo: float, hi: float, label: str) -> float:
    """Clamp a value to [lo, hi] and log a warning if it was out of range."""
    if value < lo or value > hi:
        clamped = max(lo, min(hi, value))
        logger.warning("Clamped %s: %.3f -> %.3f (bounds [%.2f, %.2f])",
                       label, value, clamped, lo, hi)
        return clamped
    return value


def _clamp_motion_step(step: MotionStep) -> MotionStep:
    """Validate and clamp a MotionStep's values to safe workspace bounds."""
    ee = step.ee_target
    ee[0] = _clamp(ee[0], 0.15, 0.75, "ee_target.x")
    ee[1] = _clamp(ee[1], -0.4, 0.4, "ee_target.y")
    ee[2] = _clamp(ee[2], 0.05, 0.8, "ee_target.z")

    wrist = step.wrist_orientation
    for i, axis in enumerate(["roll", "pitch", "yaw"]):
        wrist[i] = _clamp(wrist[i], -1.6, 1.6, f"wrist.{axis}")

    for finger, val in step.finger_config.items():
        step.finger_config[finger] = _clamp(val, 0.0, 1.0, f"finger.{finger}")

    step.duration_steps = int(_clamp(step.duration_steps, 20, 500, "duration_steps"))

    return step


def interpret_command(
    image: np.ndarray,
    command: str,
    object_info: dict | None = None,
    robot_state: dict | None = None,
) -> MotionPlan:
    """Interpret a free-form natural language command into a MotionPlan.

    The VLM generates a sequence of waypoints that the robot follows,
    enabling arbitrary tasks (not just grasping).

    Args:
        image: RGB image from camera (H, W, 3) uint8
        command: Free-form text, e.g. "pick up the object", "wave hello"
        object_info: Optional dict with object/scene metadata
        robot_state: Optional dict with ee_position, wrist_orientation,
                     finger_state for current robot pose

    Returns:
        MotionPlan with a list of MotionSteps
    """
    image_b64 = _encode_image(image)

    rs = robot_state or {}
    prompt = COMMAND_INTERPRETATION_PROMPT.format(
        command=command,
        object_info=json.dumps(object_info or {}, indent=2),
        robot_ee_pos=rs.get("ee_position", [0.3, 0.0, 0.6]),
        robot_wrist=rs.get("wrist_orientation", [0.0, -0.3, 0.0]),
        robot_fingers=json.dumps(rs.get("finger_state",
                                        {"thumb": 0.0, "index": 0.0, "middle": 0.0,
                                         "ring": 0.0, "pinky": 0.0})),
    )

    _default_fingers = {"thumb": 0.0, "index": 0.0, "middle": 0.0,
                        "ring": 0.0, "pinky": 0.0}

    try:
        response_text = _call_vlm(image_b64, prompt)
        data = _parse_json_response(response_text)

        reasoning = data.get("reasoning", "")
        if reasoning:
            logger.info("VLM reasoning: %s", reasoning)

        steps = []
        for s in data.get("steps", []):
            step = MotionStep(
                description=s.get("description", ""),
                ee_target=s.get("ee_target", [0.3, 0.0, 0.5]),
                wrist_orientation=s.get("wrist_orientation", [0.0, -0.3, 0.0]),
                finger_config=s.get("finger_config", _default_fingers),
                duration_steps=max(int(s.get("duration_steps", 100)), 20),
            )
            steps.append(_clamp_motion_step(step))

        if not steps:
            raise ValueError("VLM returned empty steps list")

        return MotionPlan(
            description=data.get("description", ""),
            steps=steps,
            reasoning=reasoning,
        )
    except Exception as e:
        logger.warning("VLM command interpretation failed: %s; using default plan", e)
        # Fallback: simple reach-forward motion so the robot visibly does something
        return MotionPlan(
            description=f"Fallback plan for: {command}",
            steps=[
                MotionStep(
                    description="Reach forward",
                    ee_target=[0.5, 0.0, 0.4],
                    duration_steps=120,
                ),
                MotionStep(
                    description="Return home",
                    ee_target=[0.3, 0.0, 0.6],
                    duration_steps=120,
                ),
            ],
        )
