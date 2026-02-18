"""Manage multiple camera views for the humanoid simulation."""

import logging

import numpy as np

logger = logging.getLogger(__name__)


class CameraManager:
    """Renders observations from ego, third-person, and wrist cameras."""

    def __init__(self, model, render_width: int = 640, render_height: int = 480,
                 obs_width: int = 256, obs_height: int = 256):
        import mujoco

        self._model = model
        self._render_w = render_width
        self._render_h = render_height
        self._obs_w = obs_width
        self._obs_h = obs_height

        # Video renderer (large, for overview)
        self._video_renderer = mujoco.Renderer(model, height=render_height, width=render_width)
        # Observation renderer (small, for model input)
        self._obs_renderer = mujoco.Renderer(model, height=obs_height, width=obs_width)

        # Resolve camera IDs
        self._cam_ids = {}
        for name in ["overview_cam", "ego_cam", "wrist_cam_left", "wrist_cam_right"]:
            try:
                self._cam_ids[name] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            except Exception:
                self._cam_ids[name] = None

    def _render(self, renderer, data, cam_id) -> np.ndarray | None:
        try:
            if cam_id is not None and cam_id >= 0:
                renderer.update_scene(data, camera=cam_id)
            else:
                renderer.update_scene(data)
            return renderer.render().copy()
        except Exception as e:
            logger.debug("Render failed for cam %s: %s", cam_id, e)
            return None

    def render_video_frame(self, data) -> np.ndarray | None:
        """Render a 640x480 frame from the overview camera for video recording."""
        return self._render(self._video_renderer, data, self._cam_ids.get("overview_cam"))

    def render_ego(self, data) -> np.ndarray | None:
        """Render 256x256 from the egocentric (head) camera."""
        return self._render(self._obs_renderer, data, self._cam_ids.get("ego_cam"))

    def render_overview(self, data) -> np.ndarray | None:
        """Render 256x256 from the overview camera (for planner/VLM)."""
        return self._render(self._obs_renderer, data, self._cam_ids.get("overview_cam"))

    def render_wrist(self, data, side: str = "right") -> np.ndarray | None:
        """Render 256x256 from a wrist camera."""
        cam_name = f"wrist_cam_{side}"
        return self._render(self._obs_renderer, data, self._cam_ids.get(cam_name))

    def get_observations_for_controller(self, data) -> dict[str, np.ndarray]:
        """Get observation images formatted for the low-level controller (OpenPI).

        Returns a dict with keys matching OpenPI's expected observation format.
        """
        obs = {}
        ego = self.render_ego(data)
        if ego is not None:
            obs["image"] = ego

        wrist = self.render_wrist(data, "right")
        if wrist is not None:
            obs["wrist_image"] = wrist

        return obs

    def get_observation_for_planner(self, data) -> np.ndarray | None:
        """Get a third-person view for the high-level VLM planner."""
        return self.render_overview(data)
