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


def _annotate_obs_image(frame: np.ndarray, model, data,
                        cam_id: int, ee_site_id, obj_body_id) -> np.ndarray:
    """Draw spatial annotations on the VLM observation image.

    Reads actual positions from MuJoCo physics state (not commanded targets).
    Projects world coordinate axes through the camera's rotation matrix so
    the arrows match what the VLM sees in the image.
    """
    import mujoco

    if frame.dtype != np.uint8:
        frame = (np.clip(frame, 0, 1) * 255).astype(np.uint8)
    pil_img = Image.fromarray(frame)
    draw = ImageDraw.Draw(pil_img)

    # --- Read actual positions from physics ---
    if ee_site_id is not None and ee_site_id >= 0:
        ee_pos = data.site_xpos[ee_site_id].copy()
    else:
        ee_pos = np.array([0.3, 0.0, 0.6])

    if obj_body_id is not None and obj_body_id >= 0:
        obj_pos = data.xpos[obj_body_id].copy()
    else:
        obj_pos = np.array([0.5, 0.0, 0.3])

    # --- Coordinate axes: project world axes through camera rotation ---
    # data.cam_xmat stores the 3x3 rotation matrix (row-major, 9 elements)
    # Columns of this matrix are the camera's local x, y, z axes in world coords
    if cam_id is not None and cam_id >= 0:
        cam_mat = data.cam_xmat[cam_id].reshape(3, 3)
        # cam_mat rows: cam_x_world, cam_y_world, cam_z_world
        # To project world axis v onto image: img_x = dot(v, cam_x), img_y = dot(v, cam_y)
        # PIL image: +x right, +y down (so negate cam_y projection)
        cam_x = cam_mat[0]  # camera right direction in world
        cam_y = cam_mat[1]  # camera up direction in world

        origin = (35, 50)
        arrow_len = 25
        axes = [
            (np.array([1, 0, 0]), "X", (255, 50, 50)),
            (np.array([0, 1, 0]), "Y", (50, 255, 50)),
            (np.array([0, 0, 1]), "Z", (50, 150, 255)),
        ]
        for world_axis, label, color in axes:
            # Project world axis onto camera image plane
            px = float(np.dot(world_axis, cam_x))
            py = float(-np.dot(world_axis, cam_y))  # negate: cam_y is up, PIL y is down
            # Normalize to fixed arrow length
            mag = max(np.sqrt(px**2 + py**2), 1e-6)
            dx = int(arrow_len * px / mag)
            dy = int(arrow_len * py / mag)
            end = (origin[0] + dx, origin[1] + dy)
            draw.line([origin, end], fill=color, width=2)
            draw.text((end[0] + 2, end[1] - 6), label, fill=color)

    # --- Text labels with actual physics positions ---
    h = pil_img.height
    ee_str = f"EE: [{ee_pos[0]:.2f}, {ee_pos[1]:.2f}, {ee_pos[2]:.2f}]"
    obj_str = f"Obj: [{obj_pos[0]:.2f}, {obj_pos[1]:.2f}, {obj_pos[2]:.2f}]"

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
    ee_site_id = _safe_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    obj_body_id = _safe_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")

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
                    # Annotate with actual physics positions and correct axes
                    obs_image = _annotate_obs_image(
                        obs_image, model, data,
                        overview_cam_id, ee_site_id, obj_body_id,
                    )

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
