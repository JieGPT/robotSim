"""CLI — run robot validation demo scenario from command line.

Demonstrates all MVP features:
  1. Load robot model from URDF
  2. Build workcell (table, obstacles)
  3. Execute PTP trajectory with collision + joint-limit checks
  4. Visualize in 3D via viser
  5. Print results summary and export as JSON
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import mujoco
import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from robot_validator.scenario import from_yaml, TaskStep, ValidationScenario
from robot_validator.robot import Robot
from robot_validator.workcell import WorkcellBuilder
from robot_validator.controller.joint_pos import JointPosPTP
from robot_validator.validation.collision import CollisionCriterion
from robot_validator.validation.kinematics import JointLimitCriterion, Severity
from robot_validator.session import SessionRunner, ValidationResult
from robot_validator.visualizer import ValidationVisualizer

app = typer.Typer(
    name="robot-validator",
    help="Industrial robot validation — verify workcells, trajectories, collisions",
    add_completion=False,
)
console = Console(file=sys.stderr)


# ── helpers ───────────────────────────────────────────────────────────────

EXAMPLES = Path(__file__).resolve().parent.parent.parent / "examples"


def _load_demo_scenario(yaml_path: str | None = None) -> ValidationScenario:
    """Load the built-in demo YAML or fall back to an in-memory scenario."""
    if yaml_path:
        return from_yaml(yaml_path)

    # Minimal in-memory scenario
    from robot_validator.scenario import (
        RobotConfig, WorkcellConfig, TaskConfig, CriterionConfig,
    )
    return ValidationScenario(
        name="demo_inline",
        robot=RobotConfig(urdf="models/fr3_panda.urdf"),
        workcell=WorkcellConfig(
            fixtures=[{"name": "table", "type": "box", "position": [0.6, 0.0, 0.0], "dimensions": [1.0, 0.8, 0.05]}],
        ),
        task=TaskConfig(
            steps=[
                TaskStep(move="PTP", target=[0.0, -0.5, 0.8, -1.5, -0.3, 1.0, 0.5], velocity=0.5, acceleration=5.0),
                TaskStep(wait=0.5),
                TaskStep(move="PTP", target=[0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 0.0], velocity=0.3, acceleration=3.0),
            ],
        ),
        criteria=[],
    )


def _build_model(scenario: ValidationScenario, urdf_base: Path):
    """Load robot + merge workcell → return (model, data, n_actuated)."""
    robot = Robot.from_urdf(str(urdf_base / scenario.robot.urdf))
    mujoco.mj_resetData(robot.model, robot.data)

    workcell = WorkcellBuilder()
    if scenario.workcell:
        for f in (scenario.workcell.fixtures or []):
            workcell.add_box(f["name"], f["position"], f["dimensions"])
        for o in (scenario.workcell.obstacles or []):
            workcell.add_box(o["name"], o["position"], o["dimensions"])

    merged = workcell.merge(robot.xml)
    model = mujoco.MjModel.from_xml_string(merged)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    return model, data, len(robot.joint_names)


def _run_ptp_step(model, data, n_act: int, step, current_qpos, dt=0.005):
    """Execute one PTP step, returning (collisions, violations, new_qpos)."""
    target = np.array(step.target, dtype=np.float64)
    if float(np.max(np.abs(target))) > 10.0:
        target = np.radians(target)

    ctrl = JointPosPTP(target, vel=float(step.velocity), acc=float(step.acceleration))
    ctrl._qpos_init = current_qpos.copy()
    ctrl._calc_total_time()

    collisions = 0
    violations = 0
    new_qpos = current_qpos.copy()

    cc = CollisionCriterion()
    jlc = JointLimitCriterion(margin=0.05)

    while not ctrl.is_done:
        new_qpos = ctrl.step(dt)
        data.qpos[:n_act] = new_qpos
        mujoco.mj_step(model, data)
        collisions += len(cc.check(model, data))
        v = jlc.check(model, data)
        violations += len([r for r in v if r.severity.value >= 1])

    return collisions, violations, new_qpos


def _render_results(results: ValidationResult) -> str:
    """Render results as a rich table."""
    table = Table(title="Validation Results", show_header=True)
    table.add_column("Step", style="cyan")
    table.add_column("Type", style="white")
    table.add_column("Status", style="bold")
    table.add_column("Collisions", style="red")
    table.add_column("Violations", style="yellow")
    table.add_column("Time (s)", style="green")

    for s in results.steps:
        status = "✅ PASS" if s.passed else "❌ FAIL"
        table.add_row(
            str(s.step_index),
            s.step_type,
            status,
            str(s.collision_count),
            str(s.joint_violations),
            f"{s.elapsed_seconds:.3f}",
        )

    console.print()
    console.print(table)

    # Summary panel
    passed = "✅ PASS" if results.passed else "❌ FAIL"
    console.print(
        Panel(
            f"Scenario: [bold]{results.name}[/bold]\n"
            f"Steps: {len(results.steps)}\n"
            f"Total collisions: {results.total_collisions}\n"
            f"Total joint violations: {results.total_joint_violations}\n"
            f"Overall: [bold]{passed}[/bold]",
            title="[bold]Summary[/bold]",
        )
    )
    console.print()


# ── CLI commands ──────────────────────────────────────────────────────────

@app.command()
def demo(
    yaml_path: str | None = typer.Option(None, "--yaml", "-y", help="Path to YAML scenario (or use built-in demo)"),
    visualize: bool = typer.Option(False, "--visualize", "-v", help="Launch viser 3D visualization"),
    dry_run: bool = typer.Option(False, "--dry-run", help="Load scenario but don't simulate"),
    export_json: bool = typer.Option(False, "--json", help="Export results as JSON to stdout"),
):
    """Run a full validation demo — robot, workcell, PTP, collision, joint limits.

    Examples:
        python demo.py                           # built-in demo
        python demo.py -y examples/mvp.yaml      # custom YAML
        python demo.py --visualize               # with 3D viewer
        python demo.py --json > results.json     # JSON export
    """
    scenario = _load_demo_scenario(yaml_path)
    urdf_base = EXAMPLES if yaml_path else (Path(yaml_path).parent if yaml_path else EXAMPLES)

    if dry_run:
        console.print(Panel(f"[bold]{scenario.name}[/bold]\n{scenario.description or ''}\nRobot: {scenario.robot.urdf}\nSteps: {len(scenario.task.steps)}", title="Scenario Loaded", border_style="blue"))
        console.print(f"\nSteps:")
        for i, s in enumerate(scenario.task.steps):
            if s.move == "PTP":
                console.print(f"  [{i}] PTP → {s.target} (v={s.velocity}, a={s.acceleration})")
            elif s.wait is not None:
                console.print(f"  [{i}] WAIT {s.wait}s")
        return

    # Build model
    model, data, n_act = _build_model(scenario, urdf_base)
    console.print(f"[bold blue]✓ Robot loaded:[/bold blue] {n_act} actuated joints")
    console.print(f"[bold blue]✓ Workcell built:[/bold blue] {model.nbody} bodies, {model.ngeom} geoms")

    # Launch viser if requested
    viz = None
    if visualize:
        viz = ValidationVisualizer(model, data)
        viz.launch()

    # Execute each task step
    from robot_validator.session import ValidationResult, StepResult
    results = ValidationResult(name=scenario.name, scenario_path=yaml_path or "demo_inline")

    current_qpos = data.qpos.copy()[:n_act]

    for i, step in enumerate(scenario.task.steps):
        if step.wait is not None:
            results.add_step(StepResult(
                step_index=i, step_type="WAIT", passed=True,
                collision_count=0, joint_violations=0,
                elapsed_seconds=float(step.wait),
                details={"type": "wait", "duration": step.wait},
            ))
            continue

        if step.move != "PTP":
            continue

        collisions, violations, current_qpos = _run_ptp_step(model, data, n_act, step, current_qpos)

        results.add_step(StepResult(
            step_index=i, step_type="PTP",
            passed=(collisions == 0 and violations == 0),
            collision_count=collisions, joint_violations=violations,
            elapsed_seconds=0,
            details={
                "type": "PTP", "target": step.target,
                "final_qpos": current_qpos.tolist(),
            },
        ))

    console.print(f"\n[bold blue]✓ Simulation complete[/bold blue] ({len(results.steps)} steps)")
    if not export_json:
        _render_results(results)

    if export_json:
        typer.echo(results.to_json())

    if not results.passed:
        raise typer.Exit(code=1)


# ── entry point ────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app()
