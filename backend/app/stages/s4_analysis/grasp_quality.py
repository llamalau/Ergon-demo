"""Grasp quality metrics."""

import numpy as np


def compute_grasp_quality(trial_results: list[dict], aggregated: dict) -> dict:
    """Compute grasp quality score and metrics.

    Evaluates based on success rate, contact forces, and lift stability.
    """
    success_count = sum(
        1 for r in trial_results
        if r.get("success", {}).get("success", False)
    )
    total = max(len(trial_results), 1)
    success_rate = success_count / total

    # Force appropriateness (not too little, not too much)
    contact = aggregated.get("contact_forces", {})
    mean_force = contact.get("mean", 0)
    max_force = contact.get("max", 0)

    # Ideal force range: 1-50N for typical objects
    force_score = 1.0
    if max_force > 100:
        force_score = max(0, 1.0 - (max_force - 100) / 200)
    elif mean_force < 0.1:
        force_score = mean_force / 0.1

    # Stability (low force variance is good)
    force_std = contact.get("std", 0)
    stability_score = max(0, 1.0 - force_std / max(mean_force + 0.01, 1))

    quality_score = (success_rate * 0.5 + force_score * 0.3 + stability_score * 0.2) * 100

    return {
        "name": "grasp_quality",
        "score": round(quality_score, 1),
        "success_rate": round(success_rate, 3),
        "mean_contact_force": round(mean_force, 3),
        "max_contact_force": round(max_force, 3),
        "force_appropriateness": round(force_score * 100, 1),
        "stability": round(stability_score * 100, 1),
    }
