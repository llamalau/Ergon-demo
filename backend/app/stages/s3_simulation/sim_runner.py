"""MuJoCo simulation runner with headless OSMesa rendering."""

import numpy as np


def run_simulation(
    mjcf_xml: str,
    agent,
    task_config: dict,
    max_steps: int = 2000,
    render: bool = True,
    render_width: int = 640,
    render_height: int = 480,
    assets: dict[str, bytes] | None = None,
    robot_assets: dict[str, bytes] | None = None,
) -> dict:
    """Run a MuJoCo simulation with the given agent and task.

    Returns telemetry data, optional rendered frames, and observation images.
    """
    import mujoco

    # Merge object and robot mesh assets
    merged_assets = {}
    if assets:
        merged_assets.update(assets)
    if robot_assets:
        merged_assets.update(robot_assets)

    model = mujoco.MjModel.from_xml_string(mjcf_xml, assets=merged_assets)
    data = mujoco.MjData(model)

    # Set up renderers
    renderer = None
    obs_renderer = None
    if render:
        renderer = mujoco.Renderer(model, height=render_height, width=render_width)
        # Smaller renderer for observation images sent to agent/VLM
        obs_renderer = mujoco.Renderer(model, height=256, width=256)

    telemetry = {
        "timesteps": [],
        "joint_positions": [],
        "joint_velocities": [],
        "contact_forces": [],
        "object_positions": [],
        "object_orientations": [],
        "gripper_positions": [],
        "rewards": [],
        "fingertip_contacts": [],
        "ee_positions": [],
        "finger_joint_angles": [],
    }
    frames = []

    # Resolve body/site IDs once
    obj_id = _safe_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
    ee_site_id = _safe_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")

    # Resolve fingertip site IDs
    fingertip_sites = {}
    for finger in ["thumb", "index", "middle", "ring", "pinky"]:
        fid = _safe_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{finger}_tip")
        if fid is not None:
            fingertip_sites[finger] = fid

    # Find overview camera ID for video rendering
    overview_cam_id = _safe_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overview_cam")
    wrist_cam_id = _safe_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam")

    # Initialize agent
    agent.reset(model, data, task_config)

    for step in range(max_steps):
        # Capture observation image for agent (every 50 steps or on demand)
        obs_image = None
        if obs_renderer and step % 50 == 0:
            obs_image = _render_camera(obs_renderer, model, data, wrist_cam_id)

        # Agent computes action (with optional observation image)
        action = agent.act(model, data, step, obs_image)

        # Apply action (if agent provides control signals)
        if action is not None and len(action) <= model.nu:
            data.ctrl[:len(action)] = action

        # Step simulation
        mujoco.mj_step(model, data)

        # Record telemetry
        telemetry["timesteps"].append(data.time)
        telemetry["joint_positions"].append(data.qpos.copy().tolist())
        telemetry["joint_velocities"].append(data.qvel.copy().tolist())

        # Contact forces
        contact_force = np.zeros(6)
        for i in range(data.ncon):
            mujoco.mj_contactForce(model, data, i, contact_force)
        telemetry["contact_forces"].append(contact_force.tolist())

        # Object tracking
        if obj_id is not None:
            telemetry["object_positions"].append(data.xpos[obj_id].tolist())
            telemetry["object_orientations"].append(data.xquat[obj_id].tolist())
        else:
            telemetry["object_positions"].append([0, 0, 0])
            telemetry["object_orientations"].append([1, 0, 0, 0])

        # End-effector position
        if ee_site_id is not None:
            telemetry["ee_positions"].append(data.site_xpos[ee_site_id].tolist())
        else:
            telemetry["ee_positions"].append([0, 0, 0])

        # Fingertip contact detection
        fingertip_contact = {}
        for finger, site_id in fingertip_sites.items():
            tip_pos = data.site_xpos[site_id]
            # Check if any contact involves a geom near this fingertip
            in_contact = False
            for ci in range(data.ncon):
                c = data.contact[ci]
                dist = np.linalg.norm(c.pos - tip_pos)
                if dist < 0.03:  # Within 3cm of fingertip
                    in_contact = True
                    break
            fingertip_contact[finger] = in_contact
        telemetry["fingertip_contacts"].append(fingertip_contact)

        # Finger joint angles (collect all non-arm joint positions)
        finger_angles = {}
        for finger in ["thumb", "index", "middle", "ring", "pinky"]:
            for suffix in ["abduction", "mcp", "pip", "dip", "rotation"]:
                jname = f"{finger}_{suffix}"
                jid = _safe_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                if jid is not None:
                    qadr = model.jnt_qposadr[jid]
                    finger_angles[jname] = float(data.qpos[qadr])
        telemetry["finger_joint_angles"].append(finger_angles)

        # Render video frame using overview camera
        if renderer and step % 4 == 0:
            frame = _render_camera(renderer, model, data, overview_cam_id)
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
    """Look up a MuJoCo name→id, returning None if not found."""
    import mujoco
    try:
        return mujoco.mj_name2id(model, obj_type, name)
    except Exception:
        return None


def _render_camera(renderer, model, data, cam_id):
    """Render from a specific camera, falling back to default if cam_id is None."""
    try:
        if cam_id is not None and cam_id >= 0:
            renderer.update_scene(data, camera=cam_id)
        else:
            renderer.update_scene(data)
        frame = renderer.render()
        return frame.copy()
    except Exception:
        return None
