from __future__ import annotations

import numpy as np

from .base import ControllerBase


class JointPosPTP(ControllerBase):
    """Point-to-point joint-space controller with trapezoidal velocity profile."""

    def __init__(self, target, vel=1.0, acc=10.0):
        super().__init__()
        self._target = np.array(target, dtype=np.float64)
        self._vel = vel
        self._acc = acc
        self._qpos_init = None
        self._elapsed = 0.0
        self._total_time = None
        self._done = False

    def init(self, model, data, target=None, **kwargs):
        """Initialize controller with MuJoCo model."""
        self._model = model
        self._data = data
        if target is not None:
            self._target = np.array(target, dtype=np.float64)
        self._qpos_init = np.array(model.qpos.copy(), dtype=np.float64)
        self._calc_total_time()
        return self

    def _calc_total_time(self):
        if self._qpos_init is None:
            return 0.0
        deltas = np.abs(self._target - self._qpos_init)
        max_delta = np.max(deltas)
        if max_delta < 1e-6:
            return 0.0
        t_acc = self._vel / self._acc
        dist_acc = self._vel * t_acc / 2.0
        if 2 * dist_acc > max_delta:
            self._total_time = np.sqrt(max_delta / self._acc)
            self._t_max_vel = self._total_time / 2
        else:
            self._total_time = 2 * t_acc + (max_delta - 2 * dist_acc) / self._vel
            self._t_max_vel = t_acc
        return self._total_time

    def step(self, dt, model=None, data=None):
        """Execute one step of joint-space interpolation."""
        if self._done:
            return self._target.copy()
        if model is not None and self._qpos_init is None:
            self._qpos_init = np.array(model.qpos.copy(), dtype=np.float64)
            self._calc_total_time()
        self._elapsed += dt
        if self._total_time <= 0 or self._elapsed >= self._total_time:
            self._done = True
            return self._target.copy()
        t = self._elapsed
        ratio = min(t / self._total_time, 1.0)
        qpos = self._qpos_init + ratio * (self._target - self._qpos_init)
        return qpos

    def reset(self):
        """Reset controller state."""
        self._qpos_init = None
        self._elapsed = 0.0
        self._total_time = None
        self._done = False
        return self

    @property
    def is_done(self):
        return self._done

    @property
    def progress(self):
        if self._total_time <= 0:
            return 1.0
        return min(self._elapsed / self._total_time, 1.0)
