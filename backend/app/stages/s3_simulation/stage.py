import json

from app.tasks.base_stage import BaseStage
from app.services.storage import download_file, upload_file
from app.stages.s3_simulation.sim_runner import run_simulation
from app.stages.s3_simulation.scripted_agent import ScriptedAgent
from app.stages.s3_simulation.recorder import save_telemetry, encode_video

TASK_TYPES = ["grasp", "reorient", "use", "transfer"]


class SimulationStage(BaseStage):
    stage_name = "simulation"
    stage_order = 2

    def run(self, context: dict) -> dict:
        job_id = context["job_id"]
        mjcf_key = context.get("mjcf_key", "")

        self.publish_progress(0.05, "Loading MJCF model")
        mjcf_xml = download_file(mjcf_key).decode()

        # Run simulation trials for each task type
        config = context.get("config", {})
        task_types = config.get("task_types", ["grasp", "reorient"])
        num_trials = config.get("num_trials", 1)

        all_results = []
        total_work = len(task_types) * num_trials
        completed = 0

        for task_type in task_types:
            for trial in range(num_trials):
                progress = 0.1 + 0.8 * (completed / max(total_work, 1))
                self.publish_progress(progress, f"Running {task_type} trial {trial + 1}")

                agent = ScriptedAgent()
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
        }

        return context
