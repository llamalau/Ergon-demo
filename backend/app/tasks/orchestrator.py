import uuid
from datetime import datetime, timezone

from celery import chain
from sqlalchemy import select

from app.core.database import SyncSession
from app.models.job import Job, JobStage, JobStatus, StageStatus, STAGE_NAMES
from app.models.upload import Upload
from app.tasks.celery_app import celery_app


def _update_stage_status(job_id: str, stage_name: str, status: StageStatus, **kwargs):
    with SyncSession() as db:
        result = db.execute(
            select(JobStage).where(
                JobStage.job_id == uuid.UUID(job_id),
                JobStage.stage_name == stage_name,
            )
        )
        stage = result.scalar_one_or_none()
        if stage:
            stage.status = status
            if status == StageStatus.RUNNING:
                stage.started_at = datetime.now(timezone.utc)
            elif status in (StageStatus.COMPLETED, StageStatus.FAILED):
                stage.completed_at = datetime.now(timezone.utc)
            for k, v in kwargs.items():
                setattr(stage, k, v)
            db.commit()


def _update_job_status(job_id: str, status: JobStatus, **kwargs):
    with SyncSession() as db:
        result = db.execute(select(Job).where(Job.id == uuid.UUID(job_id)))
        job = result.scalar_one_or_none()
        if job:
            job.status = status
            if status == JobStatus.RUNNING:
                job.started_at = datetime.now(timezone.utc)
            elif status in (JobStatus.COMPLETED, JobStatus.FAILED):
                job.completed_at = datetime.now(timezone.utc)
            for k, v in kwargs.items():
                setattr(job, k, v)
            db.commit()


@celery_app.task(bind=True, name="run_stage")
def run_stage(self, context: dict, stage_index: int) -> dict:
    job_id = context["job_id"]
    stage_name = STAGE_NAMES[stage_index]

    _update_stage_status(job_id, stage_name, StageStatus.RUNNING)

    try:
        stage_class = _get_stage_class(stage_name)
        stage = stage_class(job_id)
        result = stage.execute(context)

        _update_stage_status(
            job_id, stage_name, StageStatus.COMPLETED,
            result_data=result.get("stage_result"),
            progress=1.0,
        )
        return result
    except Exception as e:
        _update_stage_status(
            job_id, stage_name, StageStatus.FAILED,
            error_message=str(e),
        )
        _update_job_status(job_id, JobStatus.FAILED, error_message=str(e))
        raise


def _get_job_context(job_id: str) -> dict:
    """Fetch job + upload info to build the initial pipeline context."""
    with SyncSession() as db:
        result = db.execute(select(Job).where(Job.id == uuid.UUID(job_id)))
        job = result.scalar_one()
        upload_result = db.execute(select(Upload).where(Upload.id == job.upload_id))
        upload = upload_result.scalar_one()
        return {
            "job_id": job_id,
            "upload_id": str(upload.id),
            "storage_key": upload.storage_key,
            "file_format": upload.file_format,
            "environment": job.environment,
            "config": job.config or {},
            "material": (job.config or {}).get("material", "default"),
            "property_overrides": (job.config or {}).get("property_overrides"),
        }


@celery_app.task(name="start_pipeline")
def start_pipeline(job_id: str) -> dict:
    _update_job_status(job_id, JobStatus.RUNNING)
    context = _get_job_context(job_id)

    # 4-stage pipeline: Ingest -> Digital Twin -> Simulation (with retry) -> Analysis
    pipeline = chain(
        run_stage.s(0),  # CAD Ingestion
        run_stage.s(1),  # Digital Twin (build MJCF with G1 humanoid)
        run_stage.s(2),  # Simulation (two-level agent, retry loop)
        run_stage.s(3),  # Analysis (minimal success/failure summary)
    )

    result = pipeline.apply_async(args=[context])
    return {"job_id": job_id, "chain_id": str(result.id)}


@celery_app.task(name="finalize_pipeline")
def finalize_pipeline(context: dict) -> dict:
    job_id = context["job_id"]
    _update_job_status(job_id, JobStatus.COMPLETED)
    return context


def _get_stage_class(stage_name: str):
    """Import and return the stage class for a given stage name."""
    from app.stages.s1_ingestion.stage import IngestionStage
    from app.stages.s2_digital_twin.stage import DigitalTwinStage
    from app.stages.s3_simulation.stage import SimulationStage
    from app.stages.s4_analysis.stage import AnalysisStage

    stages = {
        "cad_ingestion": IngestionStage,
        "digital_twin": DigitalTwinStage,
        "simulation": SimulationStage,
        "analysis": AnalysisStage,
    }
    return stages[stage_name]
