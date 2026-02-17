"""Force analysis metrics for manipulation."""


def compute_force_metrics(trial_results: list[dict], aggregated: dict) -> dict:
    """Analyze force profiles during manipulation."""
    contact = aggregated.get("contact_forces", {})

    max_force = contact.get("max", 0)
    mean_force = contact.get("mean", 0)
    median_force = contact.get("median", 0)

    # Efficiency: ratio of useful force to total force
    # Lower max-to-mean ratio means more consistent force application
    ratio = max_force / max(mean_force, 0.001)
    efficiency = max(0, 1.0 - (ratio - 1) / 10) * 100

    # Safety: are forces within safe limits for the object?
    # Generic threshold; in practice would be material-dependent
    safety = 100.0
    if max_force > 200:
        safety = max(0, 100 - (max_force - 200) / 5)
    elif max_force > 100:
        safety = 100 - (max_force - 100) / 10

    score = (efficiency * 0.5 + safety * 0.5)

    return {
        "name": "force_analysis",
        "score": round(score, 1),
        "max_force_n": round(max_force, 3),
        "mean_force_n": round(mean_force, 3),
        "median_force_n": round(median_force, 3),
        "peak_to_mean_ratio": round(ratio, 2),
        "efficiency": round(efficiency, 1),
        "safety": round(safety, 1),
    }
