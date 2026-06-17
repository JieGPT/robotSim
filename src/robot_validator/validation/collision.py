"""Collision criterion — detect contacts between body pairs."""
from __future__ import annotations
from dataclasses import dataclass
import mujoco
import numpy as np
import re

@dataclass
class CollisionResult:
    """Single contact between two geom bodies."""
    contact_pos: np.ndarray
    body1: str          # first body name
    body2: str          # second body name
    dist: float         # penetration distance (negative = penetrating)

class CollisionCriterion:
    """Detect collisions by checking MuJoCo data.contact[] array."""

    def __init__(self, pair_patterns: list[str] = None):
        # Default: match all pairs
        self._patterns = pair_patterns or [".*", ".*"]

    def check(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[CollisionResult]:
        """Return list of contact results for matching body pairs."""
        contacts: list[CollisionResult] = []
        for i in range(data.ncon):
            c = data.contact[i]
            g1 = model.geom(c.geom[0])
            g2 = model.geom(c.geom[1])
            b1 = model.body(g1.bodyid[0]).name
            b2 = model.body(g2.bodyid[0]).name
            if self._matches(b1, b2):
                contacts.append(CollisionResult(
                    contact_pos=np.array(c.pos),
                    body1=b1,
                    body2=b2,
                    dist=float(c.dist),
                ))
        return contacts

    def _matches(self, b1: str, b2: str) -> bool:
        """Check if body names match the collision pair patterns."""
        return any(
            re.match(p1, b1) and re.match(p2, b2)
            for p1, p2 in zip(self._patterns[:-1], self._patterns[1:])
        )
