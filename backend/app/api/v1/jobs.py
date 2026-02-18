import json
import uuid

import redis
from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from app.core.config import settings
from app.core.database import get_db
from app.models.job import Job, JobStage, JobStatus, STAGE_NAMES
from app.models.upload import Upload
from app.schemas.job import JobCreate, JobResponse
from app.services.storage import download_file
from app.tasks.orchestrator import start_pipeline

router = APIRouter(prefix="/jobs", tags=["jobs"])


@router.post("", response_model=JobResponse, status_code=201)
async def create_job(body: JobCreate, db: AsyncSession = Depends(get_db)):
    # Verify upload exists
    result = await db.execute(select(Upload).where(Upload.id == body.upload_id))
    if not result.scalar_one_or_none():
        raise HTTPException(status_code=404, detail="Upload not found")

    # Merge task_description into config so the pipeline can access it
    job_config = body.config or {}
    job_config.setdefault("task_description", body.task_description)

    job = Job(
        upload_id=body.upload_id,
        environment=body.environment,
        config=job_config,
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


@router.get("/{job_id}/videos")
async def list_job_videos(job_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    """List available simulation video keys for a job."""
    result = await db.execute(
        select(Job).where(Job.id == job_id).options(selectinload(Job.stages))
    )
    job = result.scalar_one_or_none()
    if not job:
        raise HTTPException(status_code=404, detail="Job not found")

    # Find simulation stage and extract video keys from result_data
    videos = []
    for stage in job.stages:
        if stage.stage_name == "simulation" and stage.result_data:
            video_keys = stage.result_data.get("video_keys", [])
            for vk in video_keys:
                if not vk:
                    continue
                # Extract task_type and trial from the key path
                # Format: jobs/{job_id}/videos/{task_type}_trial_{index}.mp4
                filename = vk.rsplit("/", 1)[-1] if "/" in vk else vk
                parts = filename.replace(".mp4", "").rsplit("_trial_", 1)
                task_type = parts[0] if parts else "unknown"
                trial_index = int(parts[1]) if len(parts) > 1 else 0
                videos.append({
                    "task_type": task_type,
                    "trial_index": trial_index,
                    "filename": filename,
                    "video_url": f"/api/v1/jobs/{job_id}/videos/{filename}",
                })

    return videos


@router.get("/{job_id}/videos/{filename}")
async def stream_video(job_id: uuid.UUID, filename: str,
                       db: AsyncSession = Depends(get_db)):
    """Stream an MP4 simulation video from MinIO."""
    # Validate job exists
    result = await db.execute(select(Job).where(Job.id == job_id))
    job = result.scalar_one_or_none()
    if not job:
        raise HTTPException(status_code=404, detail="Job not found")

    # Construct the MinIO storage key
    storage_key = f"jobs/{job_id}/videos/{filename}"

    try:
        video_bytes = download_file(storage_key)
    except Exception:
        raise HTTPException(status_code=404, detail="Video not found")

    return StreamingResponse(
        iter([video_bytes]),
        media_type="video/mp4",
        headers={
            "Content-Disposition": f'inline; filename="{filename}"',
            "Content-Length": str(len(video_bytes)),
        },
    )


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
