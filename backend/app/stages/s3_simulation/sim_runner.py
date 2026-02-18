"""MuJoCo simulation runner for humanoid manipulation."""

import logging

import numpy as np

from app.stages.s3_simulation.camera_manager import CameraManager

logger = logging.getLogger(__name__)


def run_simulation(
    mjcf_xml: str,
    agent,
    task_config: dict,
    max_steps: int = 5000,
    render: bool = True,
    assets: dict[str, bytes] | None = None,
) -> dict:
    """Run a MuJoCo simulation with the given agent.

    Args:
        mjcf_xml: MJCF XML string describing the scene.
        agent: Agent with reset/act/is_done/evaluate_success interface.
        task_config: Task configuration dict.
        max_steps: Maximum simulation steps.
        render: Whether to record video frames.
        assets: Object mesh assets dict (robot assets loaded from disk).

    Returns:
        Dict with keys: success, num_steps, frames, telemetry.
    """
    import mujoco

    model = mujoco.MjModel.from_xml_string(mjcf_xml, assets=assets or {})
    data = mujoco.MjData(model)

    # Set up camera manager for rendering
    cameras = CameraManager(model) if render else None

    telemetry = {
        "timesteps": [],
        "qpos": [],
        "qvel": [],
        "contact_forces": [],
        "object_positions": [],
    }
    frames = []

    # Resolve object body ID
    obj_id = _safe_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")

    # Initialize agent
    agent.reset(model, data, task_config)

    step = 0
    for step in range(max_steps):
        # Agent computes action
        action = agent.act(model, data, step)

        # Apply action
        if action is not None and len(action) <= model.nu:
            data.ctrl[:len(action)] = action

        # Step simulation
        mujoco.mj_step(model, data)

        # Record telemetry (every 10 steps to save memory)
        if step % 10 == 0:
            telemetry["timesteps"].append(data.time)
            telemetry["qpos"].append(data.qpos.copy().tolist())
            telemetry["qvel"].append(data.qvel.copy().tolist())

            # Contact forces
            total_force = np.zeros(6)
            for i in range(data.ncon):
                f = np.zeros(6)
                mujoco.mj_contactForce(model, data, i, f)
                total_force += np.abs(f)
            telemetry["contact_forces"].append(total_force.tolist())

            # Object tracking
            if obj_id is not None:
                telemetry["object_positions"].append(data.xpos[obj_id].tolist())
            else:
                telemetry["object_positions"].append([0, 0, 0])

        # Render video frame (every 4 steps)
        if cameras and step % 4 == 0:
            frame = cameras.render_video_frame(data)
            if frame is not None:
                frames.append(frame)

        # Check if task is complete
        if agent.is_done(model, data, step):
            break

    # Compute success
    success = agent.evaluate_success(model, data, task_config)

    return {
        "telemetry": telemetry,
        "frames": frames,
        "num_steps": step + 1,
        "success": success,
        "final_time": data.time,
    }


def _safe_name2id(model, obj_type, name):
    """Look up a MuJoCo name->id, returning None if not found."""
    import mujoco
    try:
        return mujoco.mj_name2id(model, obj_type, name)
    except Exception:
        return None
