import json
import uuid

import redis
from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from app.core.config import settings
from app.core.database import get_db
from app.models.job import Job, JobStage, JobStatus, STAGE_NAMES
from app.models.upload import Upload
from app.schemas.job import JobCreate, JobResponse
from app.tasks.orchestrator import start_pipeline

router = APIRouter(prefix="/jobs", tags=["jobs"])


@router.post("", response_model=JobResponse, status_code=201)
async def create_job(body: JobCreate, db: AsyncSession = Depends(get_db)):
    # Verify upload exists
    result = await db.execute(select(Upload).where(Upload.id == body.upload_id))
    if not result.scalar_one_or_none():
        raise HTTPException(status_code=404, detail="Upload not found")

    job = Job(
        upload_id=body.upload_id,
        environment=body.environment,
        config=body.config,
    )
    db.add(job)
    await db.flush()

    # Create stage records
    for i, name in enumerate(STAGE_NAMES):
        stage = JobStage(job_id=job.id, stage_name=name, stage_order=i)
        db.add(stage)

    await db.commit()

    # Launch Celery pipeline
    task_result = start_pipeline.delay(str(job.id))
    job.celery_task_id = str(task_result.id)
    await db.commit()

    await db.refresh(job, ["stages"])
    return job


@router.get("/{job_id}", response_model=JobResponse)
async def get_job(job_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    result = await db.execute(
        select(Job).where(Job.id == job_id).options(selectinload(Job.stages))
    )
    job = result.scalar_one_or_none()
    if not job:
        raise HTTPException(status_code=404, detail="Job not found")
    return job


@router.get("", response_model=list[JobResponse])
async def list_jobs(db: AsyncSession = Depends(get_db)):
    result = await db.execute(
        select(Job).options(selectinload(Job.stages)).order_by(Job.created_at.desc())
    )
    return result.scalars().unique().all()


@router.websocket("/ws/{job_id}/progress")
async def job_progress_ws(websocket: WebSocket, job_id: str):
    await websocket.accept()
    r = redis.from_url(settings.REDIS_URL)
    pubsub = r.pubsub()
    pubsub.subscribe(f"job:{job_id}:progress")

    try:
        while True:
            message = pubsub.get_message(ignore_subscribe_messages=True, timeout=1.0)
            if message and message["type"] == "message":
                await websocket.send_text(message["data"].decode())
            # Check for completion
            try:
                await websocket.receive_text()
            except WebSocketDisconnect:
                break
    except WebSocketDisconnect:
        pass
    finally:
        pubsub.unsubscribe()
        pubsub.close()
        r.close()
