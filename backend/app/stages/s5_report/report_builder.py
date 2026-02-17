"""Assemble structured report JSON."""

from datetime import datetime, timezone


def build_report(
    job_id: str,
    score_data: dict,
    metrics: dict,
    recommendations: list[dict],
    geometry_info: dict | None = None,
    properties: dict | None = None,
    environment: str = "open_space",
    heatmap_key: str | None = None,
) -> dict:
    """Build the complete structured report."""
    return {
        "job_id": job_id,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "overall_score": score_data["overall_score"],
        "grade": score_data["grade"],
        "sub_scores": score_data["sub_scores"],
        "weights": score_data["weights"],
        "metrics": {
            name: {k: v for k, v in m.items() if k != "name"}
            for name, m in metrics.items()
            if isinstance(m, dict) and "name" in m
        },
        "recommendations": recommendations,
        "object_info": {
            "geometry": {
                k: geometry_info[k]
                for k in ["vertices", "faces", "volume", "surface_area", "is_watertight"]
                if geometry_info and k in geometry_info
            } if geometry_info else {},
            "properties": {
                k: properties[k]
                for k in ["mass", "material", "friction", "rigidity", "density"]
                if properties and k in properties
            } if properties else {},
        },
        "environment": environment,
        "heatmap_key": heatmap_key,
    }
