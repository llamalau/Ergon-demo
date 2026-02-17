"""Playground API: start/stop interactive simulations and WebSocket bridge."""

import asyncio
import json
import uuid

import redis
from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from app.core.config import settings
from app.core.database import get_db
from app.models.job import Job, JobStatus
from app.schemas.playground import (
    PlaygroundStartRequest,
    PlaygroundStartResponse,
    PlaygroundStopResponse,
)
from app.tasks.playground_tasks import run_interactive_sim

router = APIRouter(prefix="/playground", tags=["playground"])

# Redis key helpers
_frames_channel = lambda sid: f"playground:{sid}:frames"
_commands_list = lambda sid: f"playground:{sid}:commands"
_state_hash = lambda sid: f"playground:{sid}:state"


@router.post("/start", response_model=PlaygroundStartResponse)
async def start_playground(
    body: PlaygroundStartRequest,
    db: AsyncSession = Depends(get_db),
):
    """Launch a live interactive simulation for a completed job."""
    result = await db.execute(
        select(Job)
        .where(Job.id == body.job_id)
        .options(selectinload(Job.stages))
    )
    job = result.scalar_one_or_none()
    if not job:
        raise HTTPException(status_code=404, detail="Job not found")
    if job.status != JobStatus.COMPLETED:
        raise HTTPException(status_code=400, detail="Job must be completed to start playground")

    # Verify digital_twin stage has MJCF
    dt_stage = next(
        (s for s in job.stages if s.stage_name == "digital_twin" and s.result_data),
        None,
    )
    if not dt_stage or not dt_stage.result_data.get("mjcf_key"):
        raise HTTPException(status_code=400, detail="Job has no MJCF model")

    session_id = uuid.uuid4().hex[:16]

    # Store initial state in Redis
    r = redis.from_url(settings.REDIS_URL)
    r.hset(_state_hash(session_id), mapping={
        "job_id": str(body.job_id),
        "phase": "starting",
        "step": "0",
        "sim_time": "0.0",
    })
    r.expire(_state_hash(session_id), 3600)
    r.close()

    # Dispatch Celery task
    task = run_interactive_sim.delay(session_id, str(body.job_id))

    return PlaygroundStartResponse(
        session_id=session_id,
        celery_task_id=str(task.id),
    )


@router.post("/{session_id}/stop", response_model=PlaygroundStopResponse)
async def stop_playground(session_id: str):
    """Send a stop signal to a running playground session."""
    r = redis.from_url(settings.REDIS_URL)
    r.rpush(_commands_list(session_id), json.dumps({"type": "stop"}))
    r.close()
    return PlaygroundStopResponse(session_id=session_id)


@router.websocket("/ws/{session_id}")
async def playground_ws(websocket: WebSocket, session_id: str):
    """Bidirectional WebSocket bridge between browser and simulation worker.

    - Relays frames/status from Redis pub/sub → WebSocket
    - Relays commands from WebSocket → Redis list
    """
    await websocket.accept()

    r = redis.from_url(settings.REDIS_URL)
    pubsub = r.pubsub()
    pubsub.subscribe(_frames_channel(session_id))

    async def _relay_frames():
        """Subscribe to Redis channel and forward messages to WebSocket."""
        try:
            while True:
                msg = pubsub.get_message(ignore_subscribe_messages=True, timeout=0.05)
                if msg and msg["type"] == "message":
                    await websocket.send_text(msg["data"].decode())
                else:
                    await asyncio.sleep(0.02)
        except (WebSocketDisconnect, Exception):
            pass

    async def _relay_commands():
        """Receive commands from WebSocket and push to Redis list."""
        try:
            while True:
                text = await websocket.receive_text()
                r.rpush(_commands_list(session_id), text)
                # Acknowledge immediately
                await websocket.send_text(json.dumps({
                    "type": "command_ack",
                    "received": True,
                }))
        except (WebSocketDisconnect, Exception):
            pass

    try:
        await asyncio.gather(
            asyncio.create_task(_relay_frames()),
            asyncio.create_task(_relay_commands()),
        )
    finally:
        # Push stop so the worker exits cleanly if client disconnects
        r.rpush(_commands_list(session_id), json.dumps({"type": "stop"}))
        pubsub.unsubscribe()
        pubsub.close()
        r.close()
