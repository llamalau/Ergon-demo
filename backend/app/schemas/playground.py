import uuid

from pydantic import BaseModel


class PlaygroundStartRequest(BaseModel):
    job_id: uuid.UUID


class PlaygroundStartResponse(BaseModel):
    session_id: str
    celery_task_id: str


class PlaygroundStopResponse(BaseModel):
    session_id: str
    status: str = "stopping"
