import uuid
from datetime import datetime

from pydantic import BaseModel


class ReportResponse(BaseModel):
    id: uuid.UUID
    job_id: uuid.UUID
    overall_score: float
    sub_scores: dict
    metrics: dict
    recommendations: list
    pdf_storage_key: str | None = None
    created_at: datetime

    model_config = {"from_attributes": True}
