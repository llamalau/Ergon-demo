"""Failure mode analysis."""


def analyze_failure_modes(trial_results: list[dict], aggregated: dict) -> dict:
    """Identify and categorize failure modes across trials."""
    failures = []

    for r in trial_results:
        success = r.get("success", {})
        if success.get("success", False):
            continue

        task_type = r.get("task_type", "unknown")
        summary = r.get("summary", {})

        # Determine failure category
        max_force = summary.get("max_contact_force", 0)
        displacement = summary.get("object_displacement", 0)

        if max_force < 0.01:
            category = "no_contact"
            description = "Robot failed to make contact with object"
        elif max_force > 150:
            category = "excessive_force"
            description = "Applied excessive force, possible damage"
        elif displacement < 0.01:
            category = "failed_grasp"
            description = "Contact made but object not moved"
        else:
            category = "dropped"
            description = "Object grasped but dropped during manipulation"

        failures.append({
            "task_type": task_type,
            "trial_index": r.get("trial_index", 0),
            "category": category,
            "description": description,
        })

    # Categorize failures
    categories = {}
    for f in failures:
        cat = f["category"]
        categories[cat] = categories.get(cat, 0) + 1

    total = len(trial_results)
    failure_rate = len(failures) / max(total, 1)

    return {
        "name": "failure_modes",
        "total_failures": len(failures),
        "failure_rate": round(failure_rate, 3),
        "categories": categories,
        "details": failures[:10],  # Limit details
    }
