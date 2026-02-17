"""Task completion metrics."""


def compute_task_completion(trial_results: list[dict], aggregated: dict) -> dict:
    """Evaluate task completion across all trial types."""
    by_task = {}
    for r in trial_results:
        task_type = r.get("task_type", "unknown")
        success = r.get("success", {})
        if task_type not in by_task:
            by_task[task_type] = {"total": 0, "success": 0, "steps": []}
        by_task[task_type]["total"] += 1
        if success.get("success", False):
            by_task[task_type]["success"] += 1
        by_task[task_type]["steps"].append(r.get("num_steps", 0))

    task_scores = {}
    for task_type, data in by_task.items():
        rate = data["success"] / max(data["total"], 1)
        avg_steps = sum(data["steps"]) / max(len(data["steps"]), 1)
        # Faster completion is better (normalize against max steps)
        speed_score = max(0, 1.0 - avg_steps / 2000)
        task_scores[task_type] = {
            "success_rate": round(rate, 3),
            "avg_steps": round(avg_steps, 1),
            "speed_score": round(speed_score * 100, 1),
            "score": round((rate * 0.7 + speed_score * 0.3) * 100, 1),
        }

    overall = sum(t["score"] for t in task_scores.values()) / max(len(task_scores), 1)

    return {
        "name": "task_completion",
        "score": round(overall, 1),
        "by_task": task_scores,
        "total_trials": len(trial_results),
    }
