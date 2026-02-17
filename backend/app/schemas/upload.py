import uuid
from datetime import datetime

from pydantic import BaseModel


class UploadResponse(BaseModel):
    id: uuid.UUID
    filename: str
    original_filename: str
    content_type: str
    file_size: int
    file_format: str
    description: str | None = None
    created_at: datetime

    model_config = {"from_attributes": True}
