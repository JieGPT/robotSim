"""Session orchestrator — run scenario end-to-end."""
from __future__ import annotations

import json
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import mujoco

from .scenario import from_yaml, ValidationScenario, TaskStep, CriterionConfig
from .robot import Robot, _build_mjcf
from .workcell import WorkcellBuilder
from .controller.joint_pos import JointPosPTP
from .validation.collision import CollisionCriterion
from .validation.kinematics import JointLimitCriterion
from .visualizer import ValidationVisualizer


@dataclass
class StepResult:
    """Result for a single task step."""
    step_index: int
    step_type: str  # 'PTP' or 'WAIT'
    passed: bool
    collision_count: int
    joint_violations: int
    elapsed_seconds: float
    details: dict

    def to_dict(self) -> dict:
        d = {
            "step_index": self.step_index,
            "step_type": self.step_type,
            "passed": self.passed,
            "collision_count": self.collision_count,
            "joint_violations": self.joint_violations,
            "elapsed_seconds": round(self.elapsed_seconds, 3),
            "details": self.details,
        }
        if not self.passed:
            d["failures"] = []
            if self.collision_count > 0:
                d["failures"].append(f"{self.collision_count} collisions detected")
            if self.joint_violations > 0:
                d["failures"].append(f"{self.joint_violations} joint limit violations")
        return d


class ValidationResult:
    """Aggregated results from a full scenario run."""

    def __init__(self, name: str, scenario_path: str):
        self.name = name
        self.scenario_path = scenario_path
        self.steps: List[StepResult] = []
        self.total_collisions: int = 0
        self.total_joint_violations: int = 0
        self.passed: bool = True

    def add_step(self, step: StepResult):
        self.steps.append(step)
        self.total_collisions += step.collision_count
        self.total_joint_violations += step.joint_violations
        if not step.passed:
            self.passed = False

    @property
    def is_ok(self) -> bool:
        """Return True if all steps passed with no critical errors."""
        return self.passed

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "scenario_path": self.scenario_path,
            "passed": self.passed,
            "total_steps": len(self.steps),
            "total_collisions": self.total_collisions,
            "total_joint_violations": self.total_joint_violations,
            "steps": [s.to_dict() for s in self.steps],
        }

    def to_json(self, indent=2) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    def summary(self) -> str:
        """Print formatted summary of results."""
        lines = []
        lines.append(f"\n{'='*60}")
        lines.append(f"Validation Results: {self.name}")
        lines.append(f"{'='*60}")
        lines.append(f"Scenario: {self.scenario_path}")
        lines.append(f"Steps executed: {len(self.steps)}")
        lines.append(f"Total collisions: {self.total_collisions}")
        lines.append(f"Joint violations: {self.total_joint_violations}")
        lines.append(f"Overall: {'✅ PASS' if self.passed else '❌ FAIL'}")
        lines.append(f"{'─'*60}")

        for step in self.steps:
            status = "✅ PASS" if step.passed else "❌ FAIL"
            lines.append(
                f"Step {step.step_index} [{step.step_type}] {status}"
            )
            if step.collision_count > 0:
                lines.append(
                    f"  ⚠️  {step.collision_count} collision(s) detected"
                )
            if step.joint_violations > 0:
                lines.append(
                    f"  ⚠️  {step.joint_violations} joint limit violation(s)"
                )

        lines.append(f"{'='*60}\n")
        return "\n".join(lines)


class SessionRunner:
    """Orchestrate a complete simulation session from YAML or Scenario."""

    def __init__(self):
        self._model: Optional[mujoco.MjModel] = None
        self._data: Optional[mujoco.MjData] = None
        self._robot: Optional[Robot] = None
        self._visualizer: Optional[ValidationVisualizer] = None
        self._results: Optional[ValidationResult] = None

    def run(
        self,
        scenario_path: str | Path | ValidationScenario,
        visualize: bool = False,
        dt: float = 0.005,
    ) -> ValidationResult:
        """Run a complete validation scenario.

        Args:
            scenario_path: Path to YAML or ValidationScenario object
            visualize: If True, launch viser server for interactive viewing
            dt: Simulation timestep (default: 0.005s)

        Returns:
            ValidationResult with per-step and aggregated results
        """
        if isinstance(scenario_path, ValidationScenario):
            scenario = scenario_path
            scenario_file = scenario.name
        else:
            scenario = from_yaml(scenario_path)
            scenario_file = str(scenario_path)

        self._results = ValidationResult(
            name=scenario.name, scenario_path=scenario_file
        )

        # Resolve URDF path
        # If it's already absolute, use it directly; otherwise resolve relative to YAML
        urdf_path = scenario.robot.urdf
        if not Path(urdf_path).is_absolute():
            if isinstance(scenario_path, Path):
                base_dir = scenario_path.parent.resolve()
            else:
                base_dir = Path(scenario_path).parent.resolve()
            urdf_path = str(base_dir / scenario.robot.urdf)

        # Build robot model from URDF
        self._robot = Robot.from_urdf(
            urdf_path,
            tuple(scenario.robot.base_position),
        )
        mujoco.mj_resetData(self._robot.model, self._robot.data)

        # Build workcell and merge with robot
        workcell = WorkcellBuilder()
        if scenario.workcell:
            for f in (scenario.workcell.fixtures or []):
                workcell.add_box(f["name"], f["position"], f["dimensions"])
            for o in (scenario.workcell.obstacles or []):
                workcell.add_box(o["name"], o["position"], o["dimensions"])

        robot_xml = self._robot.xml
        merged_xml = workcell.merge(robot_xml)
        self._model = mujoco.MjModel.from_xml_string(merged_xml)
        self._data = mujoco.MjData(self._model)
        mujoco.mj_resetData(self._model, self._data)

        # Launch viser if requested
        if visualize:
            self._visualizer = ValidationVisualizer(
                self._model, self._data
            )
            self._visualizer.launch()

        # Execute each task step
        current_qpos = self._data.qpos.copy()[:self._robot.n_actuated_joints]

        for i, step in enumerate(scenario.task.steps):
            step_result = self._execute_step(
                i, step, dt, current_qpos
            )
            self._results.add_step(step_result)
            current_qpos = step_result.details.get(
                "final_qpos", current_qpos
            )

        return self._results

    def _execute_step(
        self,
        index: int,
        step: TaskStep,
        dt: float,
        qpos: np.ndarray,
    ) -> StepResult:
        """Execute a single task step and validate."""
        if step.wait is not None:
            # Wait step — hold position
            return StepResult(
                step_index=index,
                step_type="WAIT",
                passed=True,
                collision_count=0,
                joint_violations=0,
                elapsed_seconds=step.wait,
                details={"type": "wait", "duration": step.wait},
            )

        if step.target is None:
            return StepResult(
                step_index=index,
                step_type="UNKNOWN",
                passed=False,
                collision_count=0,
                joint_violations=0,
                elapsed_seconds=0,
                details={"error": "No target specified"},
            )

        # PTP trajectory
        target = np.array(step.target, dtype=np.float64)
        # Convert from degrees to radians
        if float(np.max(np.abs(target))) > 10.0:
            target = np.radians(target)

        vel = float(step.velocity) if step.velocity else 1.0
        acc = float(step.acceleration) if step.acceleration else 10.0

        controller = JointPosPTP(target, vel=vel, acc=acc)
        controller._qpos_init = qpos.copy()
        controller._calc_total_time()

        steps_executed = 0
        collisions = 0
        violations = 0

        while not controller.is_done:
            new_qpos = controller.step(dt)
            self._data.qpos[:self._robot.n_actuated_joints] = new_qpos
            mujoco.mj_step(self._model, self._data)
            steps_executed += 1

            # Check collisions
            cc = CollisionCriterion()
            contacts = cc.check(self._model, self._data)
            collisions += len(contacts)

            # Check joint limits
            jlc = JointLimitCriterion(margin=0.05)
            limit_results = jlc.check(self._model, self._data)
            warnings_errors = [
                r for r in limit_results
                if r.severity.value >= 1
            ]
            violations += len(warnings_errors)

        result = StepResult(
            step_index=index,
            step_type="PTP",
            passed=collisions == 0 and violations == 0,
            collision_count=collisions,
            joint_violations=violations,
            elapsed_seconds=float(controller._total_time),
            details={
                "type": "PTP",
                "n_steps": steps_executed,
                "target": target.tolist(),
                "final_qpos": new_qpos.tolist(),
                "duration_seconds": float(controller._total_time),
            },
        )
        return result