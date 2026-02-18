"""Simulation stage: runs the two-level agent with retry loop."""

from app.core.config import settings
from app.tasks.base_stage import BaseStage
from app.services.storage import download_file, upload_file
from app.stages.s3_simulation.sim_runner import run_simulation
from app.stages.s3_simulation.two_level_agent import TwoLevelAgent
from app.stages.s3_simulation.recorder import encode_video


class SimulationStage(BaseStage):
    stage_name = "simulation"
    stage_order = 2

    def _download_mesh_assets(self, context: dict) -> dict[str, bytes]:
        """Download object mesh files from MinIO and return as MuJoCo assets dict."""
        assets = {}

        visual_key = context.get("visual_mesh_key", "")
        if visual_key:
            assets["meshes/visual.stl"] = download_file(visual_key)

        for i, key in enumerate(context.get("collision_mesh_keys", [])):
            assets[f"meshes/collision_{i}.stl"] = download_file(key)

        return assets

    def run(self, context: dict) -> dict:
        job_id = context["job_id"]
        mjcf_key = context.get("mjcf_key", "")
        config = context.get("config", {})

        self.publish_progress(0.05, "Loading MJCF model and meshes")
        mjcf_xml = download_file(mjcf_key).decode()
        mesh_assets = self._download_mesh_assets(context)

        task_description = config.get("task_description", "pick up the object")
        max_attempts = config.get("max_attempts", settings.DEFAULT_MAX_ATTEMPTS)
        max_steps = config.get("max_steps", settings.DEFAULT_MAX_STEPS)

        agent_config = {
            "control_model_url": settings.CONTROL_MODEL_URL,
            "replan_interval": config.get("replan_interval", 16),
        }

        task_config = {
            "task_description": task_description,
            "scene_context": config.get("scene_context", ""),
            "max_steps": max_steps,
        }

        attempt_results = []

        for attempt in range(max_attempts):
            progress = 0.1 + 0.8 * (attempt / max(max_attempts, 1))
            self.publish_progress(progress,
                                  f"Attempt {attempt + 1}/{max_attempts}: {task_description}")

            agent = TwoLevelAgent(agent_config)
            try:
                result = run_simulation(
                    mjcf_xml=mjcf_xml,
                    agent=agent,
                    task_config=task_config,
                    max_steps=max_steps,
                    render=True,
                    assets=mesh_assets,
                )
            finally:
                agent.close()

            # Save video
            video_key = None
            if result["frames"]:
                video_bytes = encode_video(result["frames"])
                if video_bytes:
                    video_key = f"jobs/{job_id}/videos/attempt_{attempt}.mp4"
                    upload_file(video_key, video_bytes, "video/mp4")

            attempt_result = {
                "attempt_index": attempt,
                "num_steps": result["num_steps"],
                "success": result["success"],
                "video_key": video_key,
            }
            attempt_results.append(attempt_result)

            # Break early on success
            if result["success"].get("success", False):
                self.publish_progress(
                    0.9, f"Task succeeded on attempt {attempt + 1}")
                break

        context["simulation_results"] = attempt_results
        num_successes = sum(
            1 for r in attempt_results if r["success"].get("success", False)
        )
        context["stage_result"] = {
            "task_description": task_description,
            "total_attempts": len(attempt_results),
            "successes": num_successes,
            "success_rate": num_successes / max(len(attempt_results), 1),
            "video_keys": [r["video_key"] for r in attempt_results if r["video_key"]],
        }

        return context
