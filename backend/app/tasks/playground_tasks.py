"""Celery task that runs an interactive MuJoCo playground session."""

import logging
import uuid

from sqlalchemy import select
from sqlalchemy.orm import selectinload

from app.core.database import SyncSession
from app.models.job import Job, JobStatus
from app.services.storage import download_file
from app.tasks.celery_app import celery_app

logger = logging.getLogger(__name__)


@celery_app.task(
    bind=True,
    name="run_interactive_sim",
    soft_time_limit=3600,
    time_limit=3660,
)
def run_interactive_sim(self, session_id: str, job_id: str) -> dict:
    """Launch an interactive simulation tied to a completed job's MJCF."""

    # --- Load job from DB ---
    with SyncSession() as db:
        result = db.execute(
            select(Job)
            .where(Job.id == uuid.UUID(job_id))
            .options(selectinload(Job.stages))
        )
        job = result.scalar_one_or_none()
        if not job:
            raise ValueError(f"Job {job_id} not found")
        if job.status != JobStatus.COMPLETED:
            raise ValueError(f"Job {job_id} is not completed (status={job.status})")

        # Find digital_twin stage result_data which contains MJCF key
        dt_stage = next(
            (s for s in job.stages if s.stage_name == "digital_twin" and s.result_data),
            None,
        )
        if not dt_stage:
            raise ValueError(f"Job {job_id} has no digital_twin result data")

        result_data = dt_stage.result_data
        mjcf_key = result_data.get("mjcf_key", "")
        if not mjcf_key:
            raise ValueError(f"Job {job_id} has no MJCF key in digital_twin stage")

    # --- Download MJCF + mesh assets ---
    logger.info("Downloading MJCF and assets for playground session %s", session_id)
    mjcf_xml = download_file(mjcf_key).decode()

    # Download mesh assets (same pattern as SimulationStage)
    assets: dict[str, bytes] = {}

    visual_key = result_data.get("visual_mesh_key", "")
    if visual_key:
        assets["meshes/visual.stl"] = download_file(visual_key)

    for i, key in enumerate(result_data.get("collision_mesh_keys", [])):
        assets[f"meshes/collision_{i}.stl"] = download_file(key)

    robot_mesh_keys = result_data.get("robot_mesh_keys", {})
    for rel_path, storage_key in robot_mesh_keys.items():
        try:
            assets[rel_path] = download_file(storage_key)
        except Exception:
            pass

    # --- Run interactive simulation ---
    from app.stages.s3_simulation.interactive_runner import run_interactive_simulation

    logger.info("Starting interactive simulation for session %s (job %s)", session_id, job_id)
    run_interactive_simulation(
        session_id=session_id,
        mjcf_xml=mjcf_xml,
        assets=assets,
    )

    return {"session_id": session_id, "job_id": job_id, "status": "completed"}
