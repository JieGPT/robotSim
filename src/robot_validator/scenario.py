from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class RobotConfig:
    model: str
    base_position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    end_effector: str = ""
    joint_config: dict[str, Any] = field(default_factory=dict)


@dataclass
class WorkcellConfig:
    fixtures: list[dict[str, Any]] = field(default_factory=list)
    obstacles: list[dict[str, Any]] = field(default_factory=list)
    safety_zones: list[dict[str, Any]] = field(default_factory=list)


@dataclass
class TaskStep:
    move: str | None = None
    target: list[float] | None = None
    target_pos: list[float] | None = None
    target_euler: list[float] | None = None
    velocity: float = 1.0
    acceleration: float = 10.0
    wait: float | None = None


@dataclass
class TaskConfig:
    steps: list[TaskStep] = field(default_factory=list)


@dataclass
class CriterionConfig:
    criterion_type: str
    params: dict[str, Any] = field(default_factory=dict)


@dataclass
class ValidationScenario:
    name: str = ""
    description: str = ""
    robot: RobotConfig | None = None
    workcell: WorkcellConfig | None = None
    task: TaskConfig | None = None
    criteria: list[CriterionConfig] = field(default_factory=list)


# ── YAML deserialization ─────────────────────────────────────────────


def _parse_step(raw: dict[str, Any]) -> TaskStep:
    return TaskStep(
        move=raw.get("move"),
        target=raw.get("target"),
        target_pos=raw.get("target_pos"),
        target_euler=raw.get("target_euler"),
        velocity=float(raw["velocity"]) if "velocity" in raw else 1.0,
        acceleration=(float(raw["acceleration"]) if "acceleration" in raw else 10.0),
        wait=float(raw["wait"]) if "wait" in raw else None,
    )


def _parse_criteria(raw: list[Any]) -> list[CriterionConfig]:
    result: list[CriterionConfig] = []
    if not isinstance(raw, list):
        return result

    for entry in raw:
        if not isinstance(entry, dict):
            continue
        for key, value in entry.items():
            if isinstance(value, dict):
                params = dict(value)
                params.pop("severity", None)
                result.append(CriterionConfig(criterion_type=key, params=params))
    return result


# ── YAML serialization ───────────────────────────────────────────────


def _dump_task_step(step: TaskStep) -> dict[str, Any]:
    if step.wait is not None:
        return {"wait": step.wait}
    d: dict[str, Any] = {"move": step.move}
    if step.target is not None:
        d["target"] = step.target
    if step.target_pos is not None:
        d["target_pos"] = step.target_pos
    if step.target_euler is not None:
        d["target_euler"] = step.target_euler
    if step.velocity != 1.0:
        d["velocity"] = step.velocity
    if step.acceleration != 10.0:
        d["acceleration"] = step.acceleration
    return d


# ── Public API ───────────────────────────────────────────────────────


def from_yaml(path: str | Path) -> ValidationScenario:
    """Load a validation scenario from a YAML file."""
    path = Path(path)
    with open(path) as f:
        raw = yaml.safe_load(f)
    if not raw:
        raise ValueError(f"Empty scenario file: {path}")

    robot_raw = raw.get("robot")
    robot = None
    if robot_raw and isinstance(robot_raw, dict):
        model_key = "model" if "model" in robot_raw else "urdf"
        robot = RobotConfig(
            model=robot_raw[model_key],
            base_position=robot_raw.get("base_position", [0.0, 0.0, 0.0]),
            end_effector=robot_raw.get("end_effector", ""),
            joint_config=robot_raw.get("joint_config", {}),
        )

    wc_raw = raw.get("workcell")
    workcell = None
    if wc_raw and isinstance(wc_raw, dict):
        workcell = WorkcellConfig(
            fixtures=wc_raw.get("fixtures", []),
            obstacles=wc_raw.get("obstacles", []),
            safety_zones=wc_raw.get("safety_zones", []),
        )

    task_raw = raw.get("task")
    task_steps: list[TaskStep] = []
    if task_raw and isinstance(task_raw, dict) and "steps" in task_raw:
        for s in task_raw["steps"]:
            if isinstance(s, dict):
                task_steps.append(_parse_step(s))
    task = TaskConfig(steps=task_steps) if task_steps else TaskConfig()

    criteria = _parse_criteria(raw.get("criteria", []))

    return ValidationScenario(
        name=raw.get("name", ""),
        description=raw.get("description", ""),
        robot=robot,
        workcell=workcell,
        task=task,
        criteria=criteria,
    )


def to_yaml(scenario: ValidationScenario, path: str | Path) -> None:
    """Serialize a ValidationScenario back to YAML."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    doc: dict[str, Any] = {}
    if scenario.name:
        doc["name"] = scenario.name
    if scenario.description:
        doc["description"] = scenario.description

    if scenario.robot:
        r = scenario.robot
        doc["robot"] = {
            "model": r.model,
        }
        if r.base_position != [0.0, 0.0, 0.0]:
            doc["robot"]["base_position"] = r.base_position
        if r.end_effector:
            doc["robot"]["end_effector"] = r.end_effector
        if r.joint_config:
            doc["robot"]["joint_config"] = r.joint_config

    if scenario.workcell:
        w = scenario.workcell
        wc_doc: dict[str, list] = {}
        if w.fixtures:
            wc_doc["fixtures"] = w.fixtures
        if w.obstacles:
            wc_doc["obstacles"] = w.obstacles
        if w.safety_zones:
            wc_doc["safety_zones"] = w.safety_zones
        if wc_doc:
            doc["workcell"] = wc_doc

    if scenario.task and scenario.task.steps:
        doc["task"] = {"steps": [_dump_task_step(s) for s in scenario.task.steps]}

    if scenario.criteria:
        doc["criteria"] = [{cr.criterion_type: cr.params} for cr in scenario.criteria]

    with open(path, "w") as f:
        yaml.dump(doc, f, default_flow_style=False, sort_keys=False)


__all__ = [
    "RobotConfig",
    "TaskConfig",
    "TaskStep",
    "ValidationScenario",
    "WorkcellConfig",
    "CriterionConfig",
    "from_yaml",
    "to_yaml",
]
