import uuid
from datetime import datetime

from pydantic import BaseModel

from app.models.job import JobStatus, StageStatus


class JobCreate(BaseModel):
    upload_id: uuid.UUID
    environment: str = "open_space"
    config: dict | None = None


class JobStageResponse(BaseModel):
    id: uuid.UUID
    stage_name: str
    stage_order: int
    status: StageStatus
    progress: float
    message: str | None = None
    result_data: dict | None = None
    error_message: str | None = None
    started_at: datetime | None = None
    completed_at: datetime | None = None

    model_config = {"from_attributes": True}


class JobResponse(BaseModel):
    id: uuid.UUID
    upload_id: uuid.UUID
    status: JobStatus
    environment: str
    config: dict | None = None
    error_message: str | None = None
    celery_task_id: str | None = None
    created_at: datetime
    started_at: datetime | None = None
    completed_at: datetime | None = None
    stages: list[JobStageResponse] = []

    model_config = {"from_attributes": True}
