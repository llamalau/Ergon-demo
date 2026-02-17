import json
import uuid
from abc import ABC, abstractmethod
from datetime import datetime, timezone

import redis

from app.core.config import settings


class BaseStage(ABC):
    stage_name: str = "unknown"
    stage_order: int = 0

    def __init__(self, job_id: str):
        self.job_id = job_id
        self._redis = redis.from_url(settings.REDIS_URL)

    def publish_progress(self, progress: float, message: str = "") -> None:
        payload = json.dumps({
            "job_id": self.job_id,
            "stage_name": self.stage_name,
            "stage_order": self.stage_order,
            "progress": progress,
            "message": message,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        })
        self._redis.publish(f"job:{self.job_id}:progress", payload)

    @abstractmethod
    def run(self, context: dict) -> dict:
        """Execute the stage. Returns updated context dict."""
        ...

    def execute(self, context: dict) -> dict:
        self.publish_progress(0.0, f"Starting {self.stage_name}")
        try:
            result = self.run(context)
            self.publish_progress(1.0, f"Completed {self.stage_name}")
            return result
        except Exception as e:
            self.publish_progress(-1.0, f"Failed: {e}")
            raise
