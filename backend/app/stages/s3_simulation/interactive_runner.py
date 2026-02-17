"""Interactive simulation runner for the Playground feature.

Runs a MuJoCo simulation loop indefinitely, streaming frames over Redis
pub/sub and accepting commands via a Redis list.
"""

import base64
import io
import json
import logging
import time

import numpy as np
import redis
from PIL import Image, ImageDraw

from app.core.config import settings
from app.stages.s3_simulation.interactive_agent import InteractiveAgent

logger = logging.getLogger(__name__)

# Redis key helpers
_frames_channel = lambda sid: f"playground:{sid}:frames"
_commands_list = lambda sid: f"playground:{sid}:commands"
_state_hash = lambda sid: f"playground:{sid}:state"

SESSION_TTL = 3600  # 1 hour


def _safe_name2id(model, obj_type, name):
    import mujoco
    try:
        return mujoco.mj_name2id(model, obj_type, name)
    except Exception:
        return None


def _render_camera(renderer, model, data, cam_id):
    try:
        if cam_id is not None and cam_id >= 0:
            renderer.update_scene(data, camera=cam_id)
        else:
            renderer.update_scene(data)
        return renderer.render().copy()
    except Exception:
        return None


def _annotate_obs_image(frame: np.ndarray, ee_pos: list[float],
                        obj_pos: list[float]) -> np.ndarray:
    """Draw spatial annotations on the VLM observation image.

    Adds a coordinate axes indicator (RGB arrows for X/Y/Z) in the top-left
    corner and a text label with current EE and object positions.
    """
    if frame.dtype != np.uint8:
        frame = (np.clip(frame, 0, 1) * 255).astype(np.uint8)
    pil_img = Image.fromarray(frame)
    draw = ImageDraw.Draw(pil_img)

    # --- Coordinate axes indicator (top-left corner) ---
    origin = (30, 40)
    arrow_len = 20
    # X axis (red) — points right in image (approximating +X forward from overview cam)
    draw.line([origin, (origin[0] + arrow_len, origin[1])], fill=(255, 0, 0), width=2)
    draw.text((origin[0] + arrow_len + 2, origin[1] - 6), "X", fill=(255, 0, 0))
    # Y axis (green) — points up in image (approximating +Y left)
    draw.line([origin, (origin[0], origin[1] - arrow_len)], fill=(0, 255, 0), width=2)
    draw.text((origin[0] - 4, origin[1] - arrow_len - 14), "Y", fill=(0, 255, 0))
    # Z axis (blue) — diagonal for depth cue
    draw.line([origin, (origin[0] - 14, origin[1] + 14)], fill=(0, 128, 255), width=2)
    draw.text((origin[0] - 24, origin[1] + 14), "Z", fill=(0, 128, 255))

    # --- Text labels (bottom of image) ---
    h = pil_img.height
    ee_str = f"EE: [{ee_pos[0]:.2f}, {ee_pos[1]:.2f}, {ee_pos[2]:.2f}]"
    obj_str = f"Obj: [{obj_pos[0]:.2f}, {obj_pos[1]:.2f}, {obj_pos[2]:.2f}]"

    # Semi-transparent background for readability
    draw.rectangle([(0, h - 30), (pil_img.width, h)], fill=(0, 0, 0, 180))
    draw.text((5, h - 28), ee_str, fill=(255, 255, 255))
    draw.text((5, h - 14), obj_str, fill=(200, 200, 255))

    return np.array(pil_img)


def _frame_to_base64_jpeg(frame: np.ndarray, quality: int = 70) -> str:
    """Encode a numpy RGB frame as a base64 JPEG string."""
    from PIL import Image

    if frame.dtype != np.uint8:
        frame = (np.clip(frame, 0, 1) * 255).astype(np.uint8)
    pil = Image.fromarray(frame)
    buf = io.BytesIO()
    pil.save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def run_interactive_simulation(
    session_id: str,
    mjcf_xml: str,
    assets: dict[str, bytes] | None = None,
    render_width: int = 640,
    render_height: int = 480,
) -> None:
    """Main interactive loop — blocks until a stop command is received."""
    import mujoco

    merged_assets = assets or {}
    model = mujoco.MjModel.from_xml_string(mjcf_xml, assets=merged_assets)
    data = mujoco.MjData(model)

    renderer = mujoco.Renderer(model, height=render_height, width=render_width)

    overview_cam_id = _safe_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overview_cam")

    agent = InteractiveAgent()
    agent.reset(model, data)

    r = redis.from_url(settings.REDIS_URL)

    # Publish initial state
    r.hset(_state_hash(session_id), mapping={"phase": "idle", "step": "0", "sim_time": "0.0"})
    r.expire(_state_hash(session_id), SESSION_TTL)

    last_frame_time = 0.0
    frame_interval = 1.0 / 15  # ~15 fps wall-clock cap

    step = 0
    try:
        while True:
            # --- Check for commands every 5 steps ---
            if step % 5 == 0:
                raw = r.lpop(_commands_list(session_id))
                if raw:
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        msg = {}
                    if msg.get("type") == "stop":
                        logger.info("Stop command received for session %s", session_id)
                        break
                    elif msg.get("type") == "command":
                        text = msg.get("text", "")
                        if text:
                            agent.receive_command(text)

            # --- Agent observation (every 50 steps) ---
            # Use the overview_cam (fixed third-person view) for VLM reasoning
            obs_image = None
            if step % 50 == 0:
                obs_image = _render_camera(renderer, model, data, overview_cam_id)
                if obs_image is not None:
                    # Annotate with spatial cues for the VLM
                    ee_pos = agent._prev_ee.tolist()
                    obj_pos = agent._object_info.get("object_position", [0.5, 0.0, 0.3])
                    obs_image = _annotate_obs_image(obs_image, ee_pos, obj_pos)

            # --- Agent step ---
            action = agent.act(model, data, step, obs_image)
            if action is not None and len(action) <= model.nu:
                data.ctrl[:len(action)] = action
            mujoco.mj_step(model, data)

            # --- Publish phase changes ---
            if agent.phase_changed:
                agent.phase_changed = False
                status_msg = json.dumps({
                    "type": "status",
                    "phase": agent.phase,
                    "message": agent.status_message,
                    "step": step,
                    "sim_time": round(data.time, 3),
                })
                r.publish(_frames_channel(session_id), status_msg)
                r.hset(_state_hash(session_id), mapping={
                    "phase": agent.phase,
                    "step": str(step),
                    "sim_time": str(round(data.time, 3)),
                })

            # --- Render & publish frame (rate-limited) ---
            if step % 20 == 0:
                now = time.monotonic()
                if now - last_frame_time >= frame_interval:
                    frame = _render_camera(renderer, model, data, overview_cam_id)
                    if frame is not None:
                        b64 = _frame_to_base64_jpeg(frame)
                        frame_msg = json.dumps({
                            "type": "frame",
                            "data": b64,
                            "step": step,
                            "sim_time": round(data.time, 3),
                            "phase": agent.phase,
                        })
                        r.publish(_frames_channel(session_id), frame_msg)
                        last_frame_time = now

            # --- Refresh TTL periodically ---
            if step % 1000 == 0:
                r.expire(_state_hash(session_id), SESSION_TTL)

            step += 1

    finally:
        # Clean up
        r.hset(_state_hash(session_id), "phase", "stopped")
        r.expire(_state_hash(session_id), 60)  # keep briefly for final reads
        renderer.close()
        r.close()
        logger.info("Interactive simulation ended for session %s at step %d", session_id, step)
