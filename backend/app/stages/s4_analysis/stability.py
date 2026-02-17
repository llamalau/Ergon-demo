"""Object stability metrics during manipulation."""

import numpy as np


def compute_stability(trial_results: list[dict], aggregated: dict) -> dict:
    """Compute stability score — how well the object maintains its pose during manipulation."""
    trajectory = aggregated.get("object_trajectory", {})

    # Height consistency during grasp
    max_height = trajectory.get("max_height", 0)
    displacement = trajectory.get("total_displacement", 0)

    # Penalize excessive wobble (displacement should be intentional)
    # For grasp tasks, some displacement is expected
    displacement_score = max(0, 1.0 - abs(displacement - 0.15) / 0.5)

    # Contact force consistency
    contact = aggregated.get("contact_forces", {})
    force_std = contact.get("std", 0)
    force_mean = contact.get("mean", 0.01)
    consistency = max(0, 1.0 - force_std / max(force_mean, 0.01))

    score = (displacement_score * 0.5 + consistency * 0.5) * 100

    return {
        "name": "stability",
        "score": round(score, 1),
        "max_height": round(max_height, 4),
        "displacement": round(displacement, 4),
        "force_consistency": round(consistency * 100, 1),
    }
