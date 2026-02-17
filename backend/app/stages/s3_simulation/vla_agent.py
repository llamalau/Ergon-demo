"""Pluggable VLA (Vision-Language-Action) agent interface for future integration."""

import numpy as np
from abc import ABC, abstractmethod


class VLAAgent(ABC):
    """Abstract base class for Vision-Language-Action model agents.

    Subclass this to integrate a VLA model (e.g., RT-2, Octo, OpenVLA)
    for learned manipulation policies.
    """

    @abstractmethod
    def reset(self, model, data, task_config: dict) -> None:
        """Initialize the agent for a new episode."""
        ...

    @abstractmethod
    def act(self, model, data, step: int) -> np.ndarray | None:
        """Compute action given current observation.

        Should process the current state (and optionally a rendered image)
        through the VLA model to produce control signals.
        """
        ...

    @abstractmethod
    def is_done(self, model, data, step: int) -> bool:
        """Check if the agent considers the task complete."""
        ...

    @abstractmethod
    def evaluate_success(self, model, data, task_config: dict) -> dict:
        """Evaluate task success."""
        ...


class DummyVLAAgent(VLAAgent):
    """Placeholder VLA agent that does nothing (for testing the interface)."""

    def reset(self, model, data, task_config: dict) -> None:
        self.max_steps = task_config.get("max_steps", 500)

    def act(self, model, data, step: int) -> np.ndarray | None:
        return None  # No action

    def is_done(self, model, data, step: int) -> bool:
        return step >= self.max_steps

    def evaluate_success(self, model, data, task_config: dict) -> dict:
        return {"task_type": task_config.get("task_type", "unknown"), "success": False, "note": "VLA agent not configured"}
