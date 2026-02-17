import json

from app.tasks.base_stage import BaseStage
from app.core.config import settings
from app.services.storage import download_file, upload_file
from app.stages.s3_simulation.sim_runner import run_simulation
from app.stages.s3_simulation.scripted_agent import ScriptedAgent
from app.stages.s3_simulation.recorder import save_telemetry, encode_video

TASK_TYPES = ["grasp", "reorient", "use", "transfer"]


def create_agent(agent_type: str, config: dict):
    """Create the appropriate agent based on configuration.

    Auto-selects VLA when an API key is configured, unless explicitly
    set to "scripted".
    """
    vlm_key = getattr(settings, "VLM_API_KEY", "")
    # Auto-upgrade to VLA when key is present and agent_type isn't forced to scripted
    use_vla = (agent_type == "vla") or (vlm_key and agent_type != "scripted")
    if use_vla and vlm_key:
        from app.stages.s3_simulation.vla_agent import VLMAgent
        return VLMAgent(config)
    return ScriptedAgent()


class SimulationStage(BaseStage):
    stage_name = "simulation"
    stage_order = 2

    def _download_mesh_assets(self, context: dict) -> dict[str, bytes]:
        """Download mesh files from MinIO and return as MuJoCo assets dict.

        Keys are relative paths matching the MJCF meshdir references,
        e.g. "meshes/visual.stl", "meshes/collision_0.stl".
        """
        assets = {}

        visual_key = context.get("visual_mesh_key", "")
        if visual_key:
            assets["meshes/visual.stl"] = download_file(visual_key)

        for i, key in enumerate(context.get("collision_mesh_keys", [])):
            assets[f"meshes/collision_{i}.stl"] = download_file(key)

        return assets

    def _download_robot_assets(self, context: dict) -> dict[str, bytes]:
        """Download robot mesh assets from MinIO."""
        assets = {}
        robot_mesh_keys = context.get("robot_mesh_keys", {})
        for rel_path, storage_key in robot_mesh_keys.items():
            try:
                assets[rel_path] = download_file(storage_key)
            except Exception:
                pass  # Non-critical: fallback hand has no mesh assets
        return assets

    def run(self, context: dict) -> dict:
        job_id = context["job_id"]
        mjcf_key = context.get("mjcf_key", "")

        self.publish_progress(0.05, "Loading MJCF model and meshes")
        mjcf_xml = download_file(mjcf_key).decode()
        mesh_assets = self._download_mesh_assets(context)
        robot_assets = self._download_robot_assets(context)

        # Run simulation trials for each task type
        config = context.get("config", {})
        task_types = config.get("task_types", ["grasp", "reorient"])
        num_trials = config.get("num_trials", 1)
        agent_type = config.get("agent_type", "auto")

        all_results = []
        total_work = len(task_types) * num_trials
        completed = 0

        for task_type in task_types:
            for trial in range(num_trials):
                progress = 0.1 + 0.8 * (completed / max(total_work, 1))
                self.publish_progress(progress, f"Running {task_type} trial {trial + 1}")

                agent = create_agent(agent_type, config)
                task_config = {
                    "task_type": task_type,
                    "trial_index": trial,
                    "max_steps": config.get("max_steps", 1000),
                }

                sim_result = run_simulation(
                    mjcf_xml=mjcf_xml,
                    agent=agent,
                    task_config=task_config,
                    max_steps=task_config["max_steps"],
                    render=True,
                    assets=mesh_assets,
                    robot_assets=robot_assets,
                )

                # Save telemetry
                telemetry_bytes, summary = save_telemetry(
                    sim_result["telemetry"], job_id, trial, task_type
                )
                telemetry_key = f"jobs/{job_id}/telemetry/{task_type}_trial_{trial}.npz"
                upload_file(telemetry_key, telemetry_bytes, "application/octet-stream")

                # Save video
                video_key = None
                if sim_result["frames"]:
                    video_bytes = encode_video(sim_result["frames"])
                    if video_bytes:
                        video_key = f"jobs/{job_id}/videos/{task_type}_trial_{trial}.mp4"
                        upload_file(video_key, video_bytes, "video/mp4")

                trial_result = {
                    "task_type": task_type,
                    "trial_index": trial,
                    "num_steps": sim_result["num_steps"],
                    "success": sim_result["success"],
                    "telemetry_key": telemetry_key,
                    "video_key": video_key,
                    "summary": summary,
                }
                all_results.append(trial_result)
                completed += 1

        context["simulation_results"] = all_results
        context["stage_result"] = {
            "num_trials": len(all_results),
            "tasks_run": list(set(r["task_type"] for r in all_results)),
            "overall_success_rate": sum(1 for r in all_results if r["success"].get("success", False)) / max(len(all_results), 1),
            "agent_type": agent_type,
            "video_keys": [r["video_key"] for r in all_results if r["video_key"]],
        }

        return context
