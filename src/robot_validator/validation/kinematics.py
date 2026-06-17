from __future__ import annotations
from enum import Enum
from dataclasses import dataclass
import numpy as np
import mujoco


class Severity(Enum):
    INFO = 0
    WARNING = 1
    ERROR = 2


@dataclass
class JointLimitResult:
    body: str
    joint_name: str
    current: float
    limit: float
    margin: float
    severity: Severity


class JointLimitCriterion:
    def __init__(self, margin=0.1):
        self._margin = margin  # fraction of joint range

    def check(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[JointLimitResult]:
        """Check all joints for limit violations."""
        results: list[JointLimitResult] = []
        qpos = data.qpos

        for i in range(model.njnt):
            jnt = model.joint(i)
            # Skip FREE and BALL joints (no range limits)
            if jnt.type in (mujoco.mjtJoint.mjJNT_FREE, mujoco.mjtJoint.mjJNT_BALL):
                continue
            
            # Only process joints with range limits (HINGE, SLIDE, SWING)
            lo, hi = jnt.range
            range_val = hi - lo
            if range_val == 0:
                continue
            
            qpos_adr = jnt.qposadr[0]
            q = qpos[qpos_adr]
            margin = self._margin * range_val
            
            # Check hard violations first (beyond limits)
            if q < lo - 1e-6 or q > hi + 1e-6:
                results.append(JointLimitResult(
                    body=".",
                    joint_name=jnt.name,
                    current=float(q),
                    limit=lo if q < lo else hi,
                    margin=0.0,
                    severity=Severity.ERROR
                ))
            # Below lower limit with margin
            elif q < lo + margin:
                results.append(JointLimitResult(
                    body=".",
                    joint_name=jnt.name,
                    current=float(q),
                    limit=lo,
                    margin=float(margin),
                    severity=Severity.WARNING
                ))
            # Above upper limit with margin
            elif q > hi - margin:
                results.append(JointLimitResult(
                    body=".",
                    joint_name=jnt.name,
                    current=float(q),
                    limit=hi,
                    margin=float(margin),
                    severity=Severity.WARNING
                ))
        return results

    def _manipulability(self, model, data):
        J = np.zeros((3, model.nv))
        mujoco.mj_jacBodyCom(model, data, (0, 0, 0), J)
        # compute manipulability ellipsoid
        return float(np.linalg.det(J @ J.T))