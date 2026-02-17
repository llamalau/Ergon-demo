"""Rule-based design recommendations from metric thresholds."""


def generate_recommendations(metrics: dict, score_data: dict) -> list[dict]:
    """Generate actionable design recommendations based on analysis results."""
    recommendations = []

    # Grasp quality recommendations
    grasp = metrics.get("grasp_quality", {})
    if grasp.get("success_rate", 1) < 0.7:
        recommendations.append({
            "category": "geometry",
            "severity": "high",
            "title": "Improve graspable surfaces",
            "description": "Success rate is below 70%. Consider adding flat surfaces or grip features to improve grasp reliability.",
            "metric": "grasp_quality",
            "current_value": grasp.get("success_rate"),
        })

    if grasp.get("force_appropriateness", 100) < 50:
        recommendations.append({
            "category": "material",
            "severity": "medium",
            "title": "Review surface friction",
            "description": "Force application is suboptimal. Consider a higher-friction surface coating or texture.",
            "metric": "grasp_quality",
        })

    # Stability recommendations
    stability = metrics.get("stability", {})
    if stability.get("score", 100) < 50:
        recommendations.append({
            "category": "geometry",
            "severity": "high",
            "title": "Improve center of mass distribution",
            "description": "Object is unstable during manipulation. Consider redistributing mass or adding stabilizing features.",
            "metric": "stability",
        })

    # Force recommendations
    forces = metrics.get("force_analysis", {})
    if forces.get("safety", 100) < 70:
        recommendations.append({
            "category": "material",
            "severity": "high",
            "title": "Reinforce structure for manipulation forces",
            "description": "Peak forces exceed safe thresholds. Consider using a stronger material or reinforcing critical areas.",
            "metric": "force_analysis",
        })

    if forces.get("efficiency", 100) < 50:
        recommendations.append({
            "category": "geometry",
            "severity": "medium",
            "title": "Optimize contact surfaces for force distribution",
            "description": "Force distribution is inefficient. Add features that allow more uniform grip force application.",
            "metric": "force_analysis",
        })

    # Task completion recommendations
    completion = metrics.get("task_completion", {})
    by_task = completion.get("by_task", {})
    for task_type, data in by_task.items():
        if data.get("success_rate", 1) < 0.5:
            recommendations.append({
                "category": "design",
                "severity": "high",
                "title": f"Redesign for {task_type} task",
                "description": f"The {task_type} task has a low success rate ({data.get('success_rate', 0):.0%}). Review object geometry for this manipulation type.",
                "metric": "task_completion",
            })

    # Failure mode recommendations
    failures = metrics.get("failure_modes", {})
    categories = failures.get("categories", {})
    if categories.get("no_contact", 0) > 0:
        recommendations.append({
            "category": "geometry",
            "severity": "medium",
            "title": "Increase object visibility/accessibility",
            "description": "Robot failed to reach the object in some trials. Ensure object dimensions allow approach from multiple angles.",
            "metric": "failure_modes",
        })
    if categories.get("excessive_force", 0) > 0:
        recommendations.append({
            "category": "material",
            "severity": "high",
            "title": "Increase object durability",
            "description": "Excessive forces detected. Object may be damaged during manipulation.",
            "metric": "failure_modes",
        })

    # Overall score recommendation
    overall = score_data.get("overall_score", 0)
    if overall >= 80:
        recommendations.append({
            "category": "overall",
            "severity": "info",
            "title": "Good manipulation quality",
            "description": f"Overall score of {overall:.0f}/100 indicates the object is well-suited for robotic manipulation.",
        })
    elif overall < 40:
        recommendations.append({
            "category": "overall",
            "severity": "high",
            "title": "Significant redesign recommended",
            "description": f"Overall score of {overall:.0f}/100 is below acceptable threshold. Major geometry and material changes are recommended.",
        })

    # Sort by severity
    severity_order = {"high": 0, "medium": 1, "info": 2}
    recommendations.sort(key=lambda r: severity_order.get(r["severity"], 3))

    return recommendations
