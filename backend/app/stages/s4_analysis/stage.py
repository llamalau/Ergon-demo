"""Analysis stage: aggregate success/failure across attempts."""

import json

from app.tasks.base_stage import BaseStage
from app.services.storage import upload_file


class AnalysisStage(BaseStage):
    stage_name = "analysis"
    stage_order = 3

    def run(self, context: dict) -> dict:
        job_id = context["job_id"]
        attempt_results = context.get("simulation_results", [])
        config = context.get("config", {})
        task_description = config.get("task_description", "unknown task")

        self.publish_progress(0.3, "Aggregating attempt results")

        total = len(attempt_results)
        successes = sum(
            1 for r in attempt_results if r.get("success", {}).get("success", False)
        )

        attempts_summary = []
        for r in attempt_results:
            success_info = r.get("success", {})
            attempts_summary.append({
                "index": r.get("attempt_index", 0),
                "success": success_info.get("success", False),
                "confidence": success_info.get("confidence", 0.0),
                "reasoning": success_info.get("reasoning", ""),
                "video_key": r.get("video_key"),
                "num_steps": r.get("num_steps", 0),
                "sub_goal_results": success_info.get("sub_goal_results", []),
            })

        summary = {
            "task_description": task_description,
            "total_attempts": total,
            "successes": successes,
            "success_rate": successes / max(total, 1),
            "attempts": attempts_summary,
        }

        self.publish_progress(0.7, "Saving results")

        results_key = f"jobs/{job_id}/results.json"
        upload_file(results_key, json.dumps(summary, indent=2).encode(), "application/json")

        context["analysis_result_key"] = results_key
        context["stage_result"] = {
            "task_description": task_description,
            "total_attempts": total,
            "successes": successes,
            "success_rate": successes / max(total, 1),
        }

        return context
