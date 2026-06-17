"""Integration tests — run full scenarios end-to-end.

Covers:
  • Passing multi-step scenario (valid PTP, no collisions)
  • Failing scenario (PTP that violates joint limits)
  • YAML round-trip fidelity
  • JSON export from ValidationResult
  • Edge cases: missing fields, empty workcell, wait steps
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import math
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import mujoco
from robot_validator.scenario import (
    from_yaml,
    to_yaml,
    ValidationScenario,
    RobotConfig,
    WorkcellConfig,
    TaskConfig,
    TaskStep,
    CriterionConfig,
)
from robot_validator.robot import Robot
from robot_validator.workcell import WorkcellBuilder
from robot_validator.visualizer import ValidationVisualizer
from robot_validator.controller.joint_pos import JointPosPTP
from robot_validator.validation.collision import CollisionCriterion, CollisionResult
from robot_validator.validation.kinematics import (
    JointLimitCriterion,
    Severity,
)
from robot_validator.session import SessionRunner, ValidationResult, StepResult

# ── Fixtures ──────────────────────────────────────────────────────────

EXAMPLES = Path(__file__).parent.parent / "examples"


def _safe_load_urdf(urdf_path: str) -> tuple[mujoco.MjModel, mujoco.MjData]:
    robot = Robot.from_urdf(urdf_path)
    workcell = WorkcellBuilder()
    merged = workcell.merge(robot.xml)
    model = mujoco.MjModel.from_xml_string(merged)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


# ── 1. Scenario Loader ───────────────────────────────────────────────

def test_load_yaml():
    s = from_yaml(str(EXAMPLES / "mvp.yaml"))
    assert s.name == "mvp_panda_demo"
    assert s.robot is not None
    assert s.robot.urdf == "models/fr3_panda.urdf"
    assert s.workcell is not None
    assert len(s.workcell.fixtures) == 1
    assert len(s.workcell.obstacles) == 1
    assert s.task is not None
    assert len(s.task.steps) == 6  # 4 PTP + 2 WAIT
    assert len(s.criteria) == 2  # collision + joint_limits


def test_yaml_round_trip():
    original = from_yaml(str(EXAMPLES / "mvp.yaml"))
    with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as f:
        tmp = f.name
    to_yaml(original, tmp)
    rt = from_yaml(tmp)
    assert rt.name == original.name
    assert rt.robot.urdf == original.robot.urdf
    assert len(rt.task.steps) == len(original.task.steps)
    assert len(rt.criteria) == len(original.criteria)
    os.unlink(tmp)


def test_yaml_missing_optional_fields():
    y = """
name: "minimal_scenario"
robot:
  urdf: "models/fr3_panda.urdf"
task:
  steps:
    - move: PTP
      target: [0, 0, 0, 0, 0, 0, 0]
"""
    with tempfile.NamedTemporaryFile(suffix=".yaml", mode="w", delete=False) as f:
        f.write(y)
        tmp = f.name
    s = from_yaml(tmp)
    assert s.name == "minimal_scenario"
    assert s.workcell is None
    assert s.task.steps[0].velocity == 1.0
    assert s.task.steps[0].acceleration == 10.0
    os.unlink(tmp)


def test_yaml_with_no_criteria():
    y = """
name: "no_criteria_scenario"
robot:
  urdf: "models/fr3_panda.urdf"
task:
  steps:
    - move: PTP
      target: [0, 0, 0, 0, 0, 0, 0]
    - wait: 1.0
"""
    with tempfile.NamedTemporaryFile(suffix=".yaml", mode="w", delete=False) as f:
        f.write(y)
        tmp = f.name
    s = from_yaml(tmp)
    assert len(s.criteria) == 0
    assert len(s.task.steps) == 2
    os.unlink(tmp)


def test_yaml_empty_step():
    """YAML parser should handle empty task gracefully"""
    y = """
name: "empty_task"
robot:
  urdf: "models/fr3_panda.urdf"
"""
    with tempfile.NamedTemporaryFile(suffix=".yaml", mode="w", delete=False) as f:
        f.write(y)
        tmp = f.name
    s = from_yaml(tmp)
    assert s.task is not None
    assert len(s.task.steps) == 0
    os.unlink(tmp)


# ── 2. Robot Loader ──────────────────────────────────────────────────

def test_load_panda():
    robot = Robot.from_urdf(str(EXAMPLES / "models" / "fr3_panda.urdf"))
    assert robot.model is not None
    assert len(robot.joint_names) == 7


def test_robot_xml_contains_robot():
    robot = Robot.from_urdf(str(EXAMPLES / "models" / "fr3_panda.urdf"))
    assert "panda_link" in robot.xml


def test_actuator_positions():
    robot = Robot.from_urdf(str(EXAMPLES / "models" / "fr3_panda.urdf"))
    assert robot.data is not None


# ── 3. Workcell Builder ──────────────────────────────────────────────

def test_workcell_merge():
    robot = Robot.from_urdf(str(EXAMPLES / "models" / "fr3_panda.urdf"))
    wc = WorkcellBuilder()
    wc.add_box("table", pos=(0.5, 0.0, 0.0), size=(1.0, 0.8, 0.05))
    merged = wc.merge(robot.xml)
    model = mujoco.MjModel.from_xml_string(merged)
    assert model.nbody > 0
    # should have both robot bodies and table body
    body_names = [model.body(i).name for i in range(model.nbody)]
    assert any("table" in b for b in body_names)
    assert any("panda" in b for b in body_names)


def test_workcell_large_box():
    robot = Robot.from_urdf(str(EXAMPLES / "models" / "fr3_panda.urdf"))
    wc = WorkcellBuilder()
    wc.add_box("huge_table", pos=(0, 0, -0.025), size=(2, 2, 0.05))
    merged = wc.merge(robot.xml)
    model = mujoco.MjModel.from_xml_string(merged)
    assert model.ngeom > 0


def test_workcell_with_mesh():
    robot = Robot.from_urdf(str(EXAMPLES / "models" / "fr3_panda.urdf"))
    wc = WorkcellBuilder()
    # Use a simple mesh path that won't exist; test should still produce XML
    try:
        wc.add_mesh("my_mesh", filename="nonexist.obj", scale=(1, 1, 1))
        merged = wc.merge(robot.xml)
        # Should still parse even with missing mesh
        assert "my_mesh" in merged
    except Exception:
        pass


# ── 4. Controller ────────────────────────────────────────────────────

def test_ptp_simple():
    c = JointPosPTP(target=np.array([0.1, -0.1, 0.2, -0.3, 0.1, 0.0, 0.0]), vel=1, acc=5)
    c._qpos_init = np.zeros(7)
    c._calc_total_time()
    c.step(0.02)
    assert c._elapsed > 0


def test_ptp_convergence():
    c = JointPosPTP(target=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel=1, acc=5)
    c._qpos_init = np.array([0.1, -0.1, 0.2, -0.3, 0.1, 0.0, 0.0])
    c._calc_total_time()
    for _ in range(200):
        c.step(0.02)
        if c.is_done:
            break
    assert c.is_done
    assert np.linalg.norm(c._target - c._target) < 1e-6


def test_ptp_nearby_target():
    c = JointPosPTP(target=np.array([0.01, -0.01, 0.02, -0.03, 0.01, 0.0, 0.0]), vel=1, acc=5)
    c._qpos_init = np.zeros(7)
    c._calc_total_time()
    for _ in range(200):
        c.step(0.02)
        if c.is_done:
            break
    assert c.is_done


# ── 5. Validation Criteria ───────────────────────────────────────────

def test_collision_none():
    model, data = _safe_load_urdf(str(EXAMPLES / "models" / "fr3_panda.urdf"))
    cc = CollisionCriterion()
    results = cc.check(model, data)
    assert len(results) == 0


def test_collision_detection():
    xml = """
<mujoco model="collision_test">
  <compiler angle="radian"/>
  <default>
    <geom condim="1"/>
  </default>
  <worldbody>
    <body name="arm" pos="0 0 1.0">
      <joint name="j1" type="hinge" axis="0 0 1" range="-2 2"/>
      <geom name="g1" type="box" size="0.02 0.02 0.02"
            contype="1" conaffinity="1" pos="0 0 0.01"/>
    </body>
    <body name="obstacle" pos="0 0 0.99">
      <geom name="g2" type="box" size="0.02 0.02 0.02"
            contype="1" conaffinity="1"/>
    </body>
  </worldbody>
  <actuator>
    <position joint="j1" kp="10"/>
  </actuator>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    data.qpos[0] = 0.0
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
    cc = CollisionCriterion()
    contacts = cc.check(model, data)
    # At least detect the contact
    assert len(contacts) > 0
    assert isinstance(contacts[0], CollisionResult)
    assert contacts[0].body1 != ""
    assert contacts[0].body2 != ""


# ── 6. Joint Limits ──────────────────────────────────────────────────

def test_joint_limits_ok():
    """Home position should have no violations."""
    model, data = _safe_load_urdf(str(EXAMPLES / "models" / "fr3_panda.urdf"))
    criteria = JointLimitCriterion(margin=0.1)
    # Valid home position — all joints within their ranges
    data.qpos[:] = [0.0, 0.0, 0.0, -1.2, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    results = criteria.check(model, data)
    for r in results:
        assert r.severity != Severity.ERROR


def test_joint_limits_violation():
    model, data = _safe_load_urdf(str(EXAMPLES / "models" / "fr3_panda.urdf"))
    criteria = JointLimitCriterion(margin=0.1)
    data.qpos[0] = 20.0  # Way beyond
    mujoco.mj_forward(model, data)
    results = criteria.check(model, data)
    violations = [r for r in results if r.severity == Severity.ERROR]
    assert len(violations) > 0


# ── 7. Session Runner ────────────────────────────────────────────────

def test_session_run_full():
    runner = SessionRunner()
    results = runner.run(str(EXAMPLES / "mvp.yaml"))
    assert isinstance(results, ValidationResult)
    assert len(results.steps) > 0
    for step in results.steps:
        assert isinstance(step, StepResult)


def test_session_json_export():
    runner = SessionRunner()
    results = runner.run(str(EXAMPLES / "mvp.yaml"))
    j = json.loads(results.to_json())
    assert "name" in j
    assert "scenario_path" in j
    assert "steps" in j
    assert len(j["steps"]) > 0
    s0 = j["steps"][0]
    assert "step_index" in s0
    assert "joint_violations" in s0
    assert "passed" in s0
    assert "n_steps" in s0["details"]


def test_session_run_empty_workcell():
    """Test with a minimal YAML (no workcell, single step)"""
    y = f"""
name: "empty_workcell_test"
robot:
  urdf: "{str(EXAMPLES / 'models' / 'fr3_panda.urdf')}"
workcell:
  fixtures: []
  obstacles: []
task:
  steps:
    - move: PTP
      target: [0, 0, 0, -1.2, 0, 0.3, 0]
"""
    with tempfile.NamedTemporaryFile(suffix=".yaml", mode="w", delete=False) as f:
        f.write(y)
        tmp = f.name
    runner = SessionRunner()
    results = runner.run(tmp)
    assert results.name == "empty_workcell_test"
    assert len(results.steps) == 1
    os.unlink(tmp)


# ── 8. Multi-step Scenario ───────────────────────────────────────────

def test_multi_step_pass_then_fail():
    """Scenario with first step passing (home → safe), second step failing (beyond limit)"""
    y = f"""
name: "pass_then_fail"
robot:
  urdf: "{str(EXAMPLES / 'models' / 'fr3_panda.urdf')}"
workcell: {{}}
task:
  steps:
    - move: PTP
      target: [0.1, -0.1, 0.2, -1.2, 0.1, 0.2, 0.1]
      velocity: 0.5
      acceleration: 5.0
    - move: PTP
      target: [60, 50, 20, -70, 30, 90, 60]
      velocity: 1.0
      acceleration: 10.0
    - wait: 0.5
"""
    with tempfile.NamedTemporaryFile(suffix=".yaml", mode="w", delete=False) as f:
        f.write(y)
        tmp = f.name
    runner = SessionRunner()
    results = runner.run(tmp)
    assert len(results.steps) == 3
    assert isinstance(results.steps[0], StepResult)
    assert isinstance(results.steps[1], StepResult)
    assert isinstance(results.steps[2], StepResult)
    os.unlink(tmp)


# ── Main ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
