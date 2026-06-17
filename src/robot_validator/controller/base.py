"""Base controller interface for robot trajectory control."""
from __future__ import annotations
import mujoco
import numpy as np
from abc import ABC, abstractmethod

class ControllerBase(ABC):
    """Abstract base for all controllers."""

    def __init__(self):
        self._model = None
        self._data = None
        self._target = None
    @abstractmethod
    def is_done(self) -> bool:
        """Check if controller has reached target."""
        raise NotImplementedError

    @abstractmethod
    def init(self, model, data, target=None):
        """Initialize controller with MuJoCo model."""
        raise NotImplementedError

    @abstractmethod
    def step(self, dt=0.01):
        """Execute one timestep of control."""
        raise NotImplementedError