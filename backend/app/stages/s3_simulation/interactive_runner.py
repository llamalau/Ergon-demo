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
    obs_renderer = mujoco.Renderer(model, height=256, width=256)

    overview_cam_id = _safe_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overview_cam")
    wrist_cam_id = _safe_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam")

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
            obs_image = None
            if step % 50 == 0:
                obs_image = _render_camera(obs_renderer, model, data, wrist_cam_id)

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
        obs_renderer.close()
        r.close()
        logger.info("Interactive simulation ended for session %s at step %d", session_id, step)
