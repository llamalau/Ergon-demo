"""Calculate composite manipulation quality score."""


WEIGHTS = {
    "grasp_quality": 0.30,
    "stability": 0.20,
    "force_analysis": 0.20,
    "task_completion": 0.30,
}


def calculate_composite_score(metrics: dict) -> dict:
    """Calculate weighted composite score from individual metric scores.

    Returns overall score (0-100) and sub-scores breakdown.
    """
    sub_scores = {}
    weighted_sum = 0.0
    total_weight = 0.0

    for metric_name, weight in WEIGHTS.items():
        metric = metrics.get(metric_name, {})
        score = metric.get("score", 0)
        sub_scores[metric_name] = round(score, 1)
        weighted_sum += score * weight
        total_weight += weight

    overall = weighted_sum / max(total_weight, 0.01)

    return {
        "overall_score": round(overall, 1),
        "sub_scores": sub_scores,
        "weights": WEIGHTS,
        "grade": _score_to_grade(overall),
    }


def _score_to_grade(score: float) -> str:
    if score >= 90:
        return "A"
    elif score >= 80:
        return "B"
    elif score >= 70:
        return "C"
    elif score >= 60:
        return "D"
    else:
        return "F"
