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
) -> dict:
    """Run a MuJoCo simulation with the given agent and task.

    Returns telemetry data and optional rendered frames.
    """
    import mujoco

    model = mujoco.MjModel.from_xml_string(mjcf_xml, assets=assets or {})
    data = mujoco.MjData(model)

    # Set up renderer if requested
    renderer = None
    if render:
        renderer = mujoco.Renderer(model, height=render_height, width=render_width)

    telemetry = {
        "timesteps": [],
        "joint_positions": [],
        "joint_velocities": [],
        "contact_forces": [],
        "object_positions": [],
        "object_orientations": [],
        "gripper_positions": [],
        "rewards": [],
    }
    frames = []

    # Initialize agent
    agent.reset(model, data, task_config)

    for step in range(max_steps):
        # Agent computes action
        action = agent.act(model, data, step)

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
            c = data.contact[i]
            mujoco.mj_contactForce(model, data, i, contact_force)
        telemetry["contact_forces"].append(contact_force.tolist())

        # Object tracking (look for 'object' body)
        try:
            obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
            telemetry["object_positions"].append(data.xpos[obj_id].tolist())
            telemetry["object_orientations"].append(data.xquat[obj_id].tolist())
        except Exception:
            telemetry["object_positions"].append([0, 0, 0])
            telemetry["object_orientations"].append([1, 0, 0, 0])

        # Render frame
        if renderer and step % 4 == 0:  # Render every 4th step
            renderer.update_scene(data)
            frame = renderer.render()
            frames.append(frame.copy())

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
