"""Load and aggregate telemetry data across simulation trials."""

import io
import numpy as np

from app.services.storage import download_file


def load_telemetry(storage_key: str) -> dict[str, np.ndarray]:
    """Load telemetry from MinIO storage."""
    data = download_file(storage_key)
    npz = np.load(io.BytesIO(data))
    return {key: npz[key] for key in npz.files}


def aggregate_telemetry(trial_results: list[dict]) -> dict:
    """Aggregate telemetry across multiple trials.

    Returns combined statistics for analysis.
    """
    all_contact_forces = []
    all_object_positions = []
    all_summaries = []

    for trial in trial_results:
        key = trial.get("telemetry_key")
        if not key:
            continue

        try:
            telemetry = load_telemetry(key)
        except Exception:
            continue

        if "contact_forces" in telemetry:
            all_contact_forces.append(telemetry["contact_forces"])
        if "object_positions" in telemetry:
            all_object_positions.append(telemetry["object_positions"])
        if trial.get("summary"):
            all_summaries.append(trial["summary"])

    result = {"num_trials": len(all_summaries)}

    if all_contact_forces:
        combined_forces = np.concatenate(all_contact_forces)
        force_mags = np.linalg.norm(combined_forces[:, :3], axis=1)
        result["contact_forces"] = {
            "max": float(np.max(force_mags)),
            "mean": float(np.mean(force_mags)),
            "std": float(np.std(force_mags)),
            "median": float(np.median(force_mags)),
        }

    if all_object_positions:
        combined_positions = np.concatenate(all_object_positions)
        result["object_trajectory"] = {
            "max_height": float(np.max(combined_positions[:, 2])),
            "total_displacement": float(
                np.mean([np.linalg.norm(p[-1] - p[0]) for p in all_object_positions])
            ),
        }

    if all_summaries:
        result["avg_duration"] = float(np.mean([s.get("duration", 0) for s in all_summaries]))
        result["avg_max_contact_force"] = float(
            np.mean([s.get("max_contact_force", 0) for s in all_summaries])
        )

    return result
