"""HTTP client for the remote OpenPI (pi0) inference server.

Sends observation dicts (images + state + language prompt) to the server
and receives action chunks (arrays of joint-level actions).
"""

import base64
import io
import logging
from collections import deque

import httpx
import numpy as np

from app.core.config import settings

logger = logging.getLogger(__name__)


class LowLevelClient:
    """Client for the OpenPI inference server running on a SLURM GPU node."""

    def __init__(self, server_url: str | None = None, replan_interval: int = 16):
        self._url = (server_url or settings.CONTROL_MODEL_URL).rstrip("/")
        self._replan_interval = replan_interval
        self._action_buffer: deque[np.ndarray] = deque()
        self._steps_since_query = 0
        self._client = httpx.Client(timeout=30.0)

    def health_check(self) -> bool:
        """Verify the inference server is reachable."""
        try:
            resp = self._client.get(f"{self._url}/health")
            return resp.status_code == 200
        except Exception as e:
            logger.warning("OpenPI health check failed: %s", e)
            return False

    def infer(self, observation: dict) -> np.ndarray:
        """Send an observation to the server, get back an action chunk.

        Args:
            observation: Dict with keys like:
                - "image": np.ndarray (H, W, 3) uint8
                - "wrist_image": np.ndarray (H, W, 3) uint8
                - "state": list[float] - joint positions
                - "prompt": str - language instruction

        Returns:
            Action array of shape (chunk_length, action_dim).
        """
        payload = self._encode_observation(observation)

        try:
            resp = self._client.post(f"{self._url}/call", json=payload)
            resp.raise_for_status()
            data = resp.json()
            actions = np.array(data["actions"], dtype=np.float64)
            return actions
        except Exception as e:
            logger.error("OpenPI inference failed: %s", e)
            raise

    def get_next_action(self, observation: dict) -> np.ndarray | None:
        """Get the next action using action chunking.

        Queries the server every `replan_interval` steps. Between queries,
        replays buffered actions from the last chunk.

        Returns a single action (1D array) or None if buffer is empty.
        """
        if not self._action_buffer or self._steps_since_query >= self._replan_interval:
            try:
                action_chunk = self.infer(observation)
                self._action_buffer = deque(action_chunk)
                self._steps_since_query = 0
            except Exception:
                # On failure, return None so caller can use fallback
                return None

        self._steps_since_query += 1

        if self._action_buffer:
            return self._action_buffer.popleft()
        return None

    def reset(self):
        """Clear the action buffer (e.g. on new sub-goal)."""
        self._action_buffer.clear()
        self._steps_since_query = 0

    def close(self):
        self._client.close()

    @staticmethod
    def _encode_observation(obs: dict) -> dict:
        """Encode observation dict to JSON-serializable format."""
        payload = {}

        for key, value in obs.items():
            if isinstance(value, np.ndarray) and value.ndim == 3:
                # Image: encode as base64 PNG
                from PIL import Image
                img = Image.fromarray(value.astype(np.uint8))
                buf = io.BytesIO()
                img.save(buf, format="PNG")
                buf.seek(0)
                payload[key] = base64.b64encode(buf.read()).decode("utf-8")
            elif isinstance(value, np.ndarray):
                payload[key] = value.tolist()
            elif isinstance(value, (list, str, int, float, bool)):
                payload[key] = value
            else:
                payload[key] = str(value)

        return payload
