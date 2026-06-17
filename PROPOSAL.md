# Industrial Robot Validation System — Proposal & Implementation Plan

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Repo Analysis & Technology Inventory](#2-repo-analysis--technology-inventory)
3. [System Vision](#3-system-vision)
4. [Core Capabilities](#4-core-capabilities)
5. [System Architecture](#5-system-architecture)
6. [Module Design](#6-module-design)
7. [BOP (Bill of Process) Integration](#7-bop-bill-of-process-integration)
8. [Data Flow](#8-data-flow)
9. [Implementation Phases](#9-implementation-phases)
10. [Key Design Decisions](#10-key-design-decisions)
11. [Validation Criteria Catalog](#11-validation-criteria-catalog)
12. [Example Scenario Walkthrough](#12-example-scenario-walkthrough)
13. [Risks & Mitigations](#13-risks--mitigations)
14. [Timeline & Milestones](#14-timeline--milestones)
15. [Appendix: Repo Deep-Dive](#15-appendix-repo-deep-dive)

---

## 1. Executive Summary

This proposal outlines a system for **industrial robot validation** that combines physics-accurate simulation with interactive 3D visualization and static reporting. The system leverages two existing Python/web frameworks in the workspace:

| Framework | Role |
|---|---|
| **mjswan** | MuJoCo physics engine, ONNX policy execution, static site bundling, MDP framework |
| **viser** | Real-time 3D visualization, GUI building blocks, URDF loading, scene interaction |

The target user is an **industrial robotics engineer** who needs to validate robot workcell designs (reachability, collision safety, cycle time, kinematic limits) before deploying to production. The system delivers two modes:

- **Live interactive sessions** (viser WebSocket) — tweak scenarios, get immediate feedback.
- **Static validation reports** (mjswan build) — shareable HTML pages with embedded simulation replay.

---

## 2. Repo Analysis & Technology Inventory

### 2.1 mjswan

| Aspect | Detail |
|---|---|
| **Purpose** | Package MuJoCo simulations + ONNX policy control into browser-based static sites |
| **License** | Apache-2.0 |
| **Python** | 3.10–3.12 |
| **Key deps** | mujoco 3.8.1, onnx, nodeenv, rich, typer, wandb |
| **Build** | Hatchling |

**Source modules:**

| Module | File | Purpose |
|---|---|---|
| Builder | `src/mjswan/builder.py` | Fluent entry point: `Builder → ProjectHandle → SceneHandle → PolicyHandle` |
| Scene | `src/mjswan/scene.py` | MuJoCo scene config, model/spec compilation, observation joint resolution |
| Project | `src/mjswan/project.py` | ProjectConfig / ProjectHandle — groups scenes |
| Policy | `src/mjswan/policy.py` | ONNX model config, command terms, observation/action/termination configs |
| Motion | `src/mjswan/motion.py` | Trajectory motion configs (reference motions for policy tracking) |
| Command | `src/mjswan/command.py` | UI input terms: `SliderConfig`, `ButtonConfig`, `CheckboxConfig`, `velocity_command` |
| Splat | `src/mjswan/splat.py` | Gaussian splatting scene overlays |
| MDP | `src/mjswan/envs/mdp/` | Actions (joint, muscle), observations, events, terminations |
| Managers | `src/mjswan/managers/` | ObservationManager, EventManager, ActionManager, TerminationManager |
| CLI | `src/mjswan/_cli.py` | Typer: `mjswan build`, `mjswan serve`, `mjswan mjlab`, etc. |
| Frontend | `src/mjswan/template/` | Vite + React + three.js + mujoco-wasm + onnxruntime-web |
| Adapters | `src/mjswan/adapters/` | mjlab compatibility layer |

**Key capabilities for validation:**
- Full MuJoCo physics (collision detection, dynamics, kinematics)
- ONNX policy inference in browser
- Scene composition (multiple geoms, bodies, joints)
- Build → static site pipeline (zip-deflate bundling)
- MDP framework for structured robot tasks (obs/act/term)

### 2.2 viser

| Aspect | Detail |
|---|---|
| **Purpose** | General-purpose 3D visualization for CV and robotics via WebSocket |
| **License** | MIT |
| **Python** | 3.8–3.14 |
| **Key deps** | websockets, numpy, msgspec, imageio, trimesh, zstandard, requests |
| **Build** | Hatchling |

**Source modules:**

| Module | File | Purpose |
|---|---|---|
| Scene API | `src/viser/_scene_api.py` | Add meshes, batched meshes, point clouds, frames, lights, splines, GLB, Gaussian splats, transform gizmos, 3D GUI containers, etc. |
| Scene Handles | `src/viser/_scene_handles.py` | Handle classes with on_click, on_drag, set_properties |
| GUI API | `src/viser/_gui_api.py` | Buttons, sliders, checkboxes, text inputs, dropdowns, folders, charts (Plotly/uPlot), modals, tabs, etc. |
| GUI Handles | `src/viser/_gui_handles.py` | Handle classes with value changes, hide/show, enable/disable |
| Transforms | `src/viser/transforms/` | SE(3), SO(3), SE(2), SO(2) Lie groups with exp/log, adjoint, sample_uniform |
| Extras | `src/viser/extras/` | `ViserUrdf` (URDF loader), Colmap, Record3D |
| Messages | `src/viser/_messages.py` | WebSocket protocol messages |
| Client | `src/viser/client/` | Web-based frontend (TypeScript) |

**Key capabilities for validation:**
- Rich 3D primitives (meshes, lines, arrows, frames, point clouds)
- URDF loading via `ViserUrdf` (uses `yourdfpy` under the hood)
- Interactive GUI (sliders for joint angles, buttons for commands)
- Scene interaction (click to select, transform gizmos, drag events)
- Programmatic camera control and rendering
- WebSocket-based real-time communication

### 2.3 Complementary Analysis

| Need | mjswan provides | viser provides | Gap to fill |
|---|---|---|---|---|
| Physics simulation | ✅ MuJoCo engine | ❌ | — |
| Robot model loading | ✅ MuJoCo XML | ✅ URDF (`ViserUrdf`) | Bridge URDF → MuJoCo XML |
| 3D visualization | ✅ three.js (static) | ✅ WebSocket live | — |
| GUI controls | ✅ Limited UI (slider/button/checkbox) | ✅ Rich GUI toolkit | Use viser for authoring, mjswan for reports |
| Static reporting | ✅ Builder → static site | ❌ | — |
| Real-time interaction | ❌ (static after build) | ✅ WebSocket live | Use viser for live sessions |
| Collision detection | ✅ MuJoCo | ❌ | Expose MuJoCo collision data |
| Trajectory control | ❌ (designed for ONNX policies) | ❌ | Build `controller/` layer |
| IK solving | ❌ | ❌ | Build lightweight IK |
| Validation framework | ❌ (no criteria system) | ❌ | Build `validation/` package |
| BOP parsing | ❌ | ❌ | Build `process/` package (parser + location resolver + compiler) |

---

## 3. System Vision

> *"Define a robot workcell scenario once, validate it interactively in 3D, then export a shareable report — all from Python, running in the browser."*

### 3.1 User Personas

| Persona | Need | Uses |
|---|---|---|
| Robotics Engineer | Validate robot placement, reachability | Interactive session, batch validation |
| Manufacturing Planner | Verify cycle time, collision safety | Static reports, CI/CD pipeline |
| Safety Officer | Check safety distances, force limits | Compliance reports |
| Application Engineer | Demo feasibility to customer | Interactive session + export |

### 3.2 Workflow

```
┌──────────┐   ┌───────────┐   ┌───────────┐   ┌──────────┐
│  Define  │──▶│ Simulate  │──▶│ Validate  │──▶│  Report  │
│ Scenario │   │  (MuJoCo) │   │ (Criteria)│   │  (HTML)  │
└──────────┘   └───────────┘   └───────────┘   └──────────┘
      │              │               │               │
      ▼              ▼               ▼               ▼
  YAML/JSON     viser live      Pass/Fail       mjswan static
  config file   3D viewer       per criterion   web report
```

---

## 4. Core Capabilities

| # | Capability | Primary Tool | Description |
|---|---|---|---|---|
| 1 | Physics simulation | mjswan / MuJoCo | Accurate collision, dynamics, kinematics |
| 2 | Robot model loading | viser `ViserUrdf` + bridge | Load any URDF → MuJoCo XML + visual in viser |
| 3 | Workcell scene definition | mjswan `SceneHandle` | Conveyors, fixtures, obstacles, safety zones |
| 4 | BOP parsing & compilation | New `process/` package | Parse Bill of Process (XML/JSON/STEP-NC) → robot command sequence |
| 5 | Controller execution | New `controller/` package | PTP/LIN/CIRC trajectories, joint control, IK |
| 6 | ONNX policy execution | mjswan `PolicyHandle` | Learned policies via onnxruntime-web |
| 7 | Interactive 3D inspection | viser scene API | Meshes, point clouds, frames, collision overlay |
| 8 | GUI controls | viser GUI API | Joint sliders, reset/record buttons, status displays |
| 9 | Validation criteria | New `validation/` package | Collision, reachability, cycle time, joint limits |
| 10 | Static reporting | mjswan `Builder.build()` | Shareable HTML with embedded simulation |
| 11 | Live validation sessions | viser WebSocket | Real-time feedback, scenario tweaking |
| 12 | Batch / CI validation | New `session.py` + CLI | Headless validation for CI pipelines |
| 13 | Scenario persistence | New `scenario.py` | Save/load as YAML/JSON |

---

## 5. System Architecture

```
┌────────────────────────────────────────────────────────────────────────┐
│                      Input / Process Layer                              │
│  ┌──────────────┐  ┌──────────────────┐  ┌─────────────────────────┐  │
│  │  BOP Parser  │──│ Location         │──│  Sequence Compiler      │  │
│  │  (XML/JSON/  │  │ Resolver         │  │  BOP → robot commands   │  │
│  │   STEP-NC)   │  │ (logical → 3D)   │  │  (ordered Task)         │  │
│  └──────┬───────┘  └────────┬─────────┘  └───────────┬─────────────┘  │
│         │                  │                          │                │
│         └──────────────────┴──────────────────────────┘                │
│                               │                                        │
├───────────────────────────────┼────────────────────────────────────────┤
│                     Validation Layer                                   │
│  ┌──────────────┐  ┌──────────────────┐  ┌──────────────────────┐    │
│  │  Scenario    │  │ Validation       │  │  Report Generator    │    │
│  │  Definition  │  │ Criteria Engine  │  │  (mjswan Builder)    │    │
│  │  (YAML/JSON) │  │ (chain + aggregate)│  │  → static site      │    │
│  └──────┬───────┘  └────────┬─────────┘  └──────────┬───────────┘    │
├─────────┼───────────────────┼───────────────────────┼────────────────┤
│         │     Simulation Layer                       │                │
│  ┌──────┴───────────────────┴───────────────────────┴─────────────┐  │
│  │                 mjswan Python API                                │  │
│  │  SceneHandle · PolicyHandle · MotionHandle                      │  │
│  │  MDP (observations/actions/terminations) · Managers             │  │
│  └──────────────────────────┬─────────────────────────────────────┘  │
│                             │                                        │
│  ┌──────────────────────────┴─────────────────────────────────────┐  │
│  │                   MuJoCo Physics Engine                         │  │
│  │  Forward dynamics · Collision detection · Kinematics            │  │
│  │  mj_step() · mj_collide() · mj_forward() · mj_kinematics()      │  │
│  └────────────────────────────────────────────────────────────────┘  │
├──────────────────────────────────────────────────────────────────────┤
│                      Controller Layer (NEW)                           │
│  ┌─────────┐  ┌────────┐  ┌────────────┐  ┌───────────────────┐    │
│  │ Joint   │  │  IK    │  │ Trajectory │  │  ONNX Policy      │    │
│  │ Control │  │ Solver │  │ PTP/LIN    │  │  Wrapper          │    │
│  │         │  │        │  │ /CIRC      │  │  (→ PolicyHandle) │    │
│  └─────────┘  └────────┘  └────────────┘  └───────────────────┘    │
├──────────────────────────────────────────────────────────────────────┤
│                      Visualization Layer                              │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │              viser Server (WebSocket-based)                     │  │
│  │  Scene API (meshes/frames/lines) · GUI API (sliders/buttons)    │  │
│  │  ViserUrdf · Transforms · Camera control · Selection/gizmos     │  │
│  └────────────────────────────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │         mjswan Template (Static site, for reports)              │  │
│  │  React + three.js + mujoco-wasm + onnxruntime-web               │  │
│  └────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

### 5.1 Component Responsibilities

| Component | Owner | Responsibilities |
|---|---|---|---|
| `scenario.py` | New | Define robot, workcell, tasks, validation criteria as a single config object |
| `workcell.py` | New | Programmatic construction of MuJoCo workcell (fixtures, conveyors, fences) |
| `robot.py` | New | URDF → MuJoCo conversion, joint configuration, kinematics chain introspection |
| `process/` | New | **BOP parsing**: `bop.py` (data model), `operation.py` (motion profiles per op type), `location_resolver.py` (logical → 3D), `sequence_compiler.py` (BOP → ordered Task) |
| `validation/` | New | Criteria engine: pluggable pass/fail checks with severity levels |
| `controller/` | New | Trajectory generation, IK, joint control, ONNX policy adapter |
| `visualizer.py` | New | viser integration: scene setup, GUI panels, collision overlay |
| `reporter.py` | New | mjswan `Builder` integration: inject validation results as static site |
| `session.py` | New | Orchestrate: load scenario → simulate → validate → produce outputs |
| `cli.py` | New | Typer-based command line |
| mjswan | Existing | MuJoCo physics, ONNX runtime, static site bundling |
| viser | Existing | Live 3D rendering, GUI, UI interaction, WebSocket transport |

---

## 6. Module Design

### 6.1 Package Structure

```
robot_validator/
├── __init__.py
│
├── scenario.py              # Scenario definition & serialization
├── workcell.py              # Workcell builder (fixtures, obstacles, zones)
├── robot.py                 # Robot model loader (URDF ↔ MuJoCo)
│
├── process/                            # ← NEW: BOP Integration
│   ├── __init__.py
│   ├── bop.py                          # BOP data model + parser (XML/JSON/STEP-NC/CSV)
│   ├── operation.py                    # Operation types + motion profile definitions
│   ├── location_resolver.py            # Map logical location names → 3D poses in workcell
│   └── sequence_compiler.py            # BOP → ordered robot command sequence
│
├── validation/
│   ├── __init__.py
│   ├── criteria.py          # Base ValidationCriterion, composite checks
│   ├── collision.py         # Self-collision & environment collision
│   ├── reachability.py      # End-effector pose reachability
│   ├── cycle_time.py        # Trajectory cycle time estimation
│   ├── kinematics.py        # Joint limit proximity, singularity detection
│   ├── safety.py            # Safety distance, force/power estimation
│   └── workspace.py         # Workspace volume coverage analysis
│
├── controller/
│   ├── __init__.py
│   ├── base.py              # Base controller interface
│   ├── joint_pos.py         # Joint position control (smooth interpolation)
│   ├── joint_vel.py         # Joint velocity control
│   ├── ik.py                # Inverse kinematics solver (CCD / MuJoCo-based)
│   ├── trajectory.py        # Trajectory primitives: PTP, LIN, CIRC
│   ├── trajectory_follower.py  # Execute trajectory step-by-step on MuJoCo
│   └── onnx_policy.py       # ONNX policy adapter → mjswan PolicyHandle
│
├── visualizer.py            # viser integration (live 3D + GUI panels)
├── reporter.py              # mjswan Builder integration (static reports)
├── session.py               # Validation session orchestrator
└── cli.py                   # Typer CLI
```

### 6.2 Key Interfaces

#### Scenario (`scenario.py`)

```python
@dataclass
class ValidationScenario:
    name: str
    robot: RobotConfig
    workcell: WorkcellConfig
    tasks: list[TaskConfig]
    criteria: list[ValidationCriterion]

    @classmethod
    def from_yaml(cls, path: Path) -> ValidationScenario: ...
    def to_yaml(self, path: Path) -> None: ...

    def validate(self) -> ValidationResult: ...
    def visualize(self) -> VisualizerSession: ...
    def report(self, output_dir: Path) -> Path: ...
```

#### Validation Criteria (`validation/criteria.py`)

```python
@dataclass
class ValidationCriterion:
    name: str
    description: str
    severity: Literal["error", "warning", "info"]

    def check(self, model: mujoco.MjModel, data: mujoco.MjData) -> CriterionResult:
        ...

@dataclass
class CriterionResult:
    passed: bool
    score: float          # 0.0 (fail) to 1.0 (pass)
    details: str
    data: dict[str, Any]  # Optional debug/visualization data

class CompositeCriterion(ValidationCriterion):
    """AND/OR/NOT combinator for criteria."""
    operator: Literal["and", "or", "not"]
    children: list[ValidationCriterion]
```

#### Controller Interface (`controller/base.py`)

```python
class Controller(ABC):
    @abstractmethod
    def init(self, model: mujoco.MjModel, data: mujoco.MjData) -> None: ...

    @abstractmethod
    def step(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
        """Return control signal (nu-length array).""" ...

    @property
    @abstractmethod
    def is_done(self) -> bool: ...

    @abstractmethod
    def reset(self) -> None: ...
```

#### Trajectory Primitives (`controller/trajectory.py`)

```python
@dataclass
class PTP:
    """Point-to-point: joint-space interpolation."""
    target: np.ndarray       # Joint position target
    velocity: float = 0.5    # rad/s
    acceleration: float = 1.0  # rad/s²
    blend_radius: float = 0.0  # For corner blending

@dataclass
class LIN:
    """Linear: Cartesian linear motion with IK."""
    target_pose: np.ndarray  # 4x4 homogeneous
    velocity: float = 0.5    # m/s
    acceleration: float = 1.0  # m/s²

@dataclass
class CIRC:
    """Circular: arc through three points."""
    via_point: np.ndarray    # 4x4 intermediate pose
    end_point: np.ndarray    # 4x4 end pose
    velocity: float = 0.5
```

#### Visualizer (`visualizer.py`)

```python
class ValidationVisualizer:
    def __init__(self, robot: Robot, workcell: Workcell): ...

    def launch(self, host: str = "0.0.0.0", port: int = 8080) -> None:
        """Start viser server with:
        - 3D scene: robot, workcell, collision meshes, reachability volume
        - GUI panel: joint sliders, criterion toggles, status display
        - Overlays: collision contact points, trajectory path, safety zones
        """ ...

    def highlight_collisions(self, contacts: list[Contact]) -> None: ...
    def show_reachability_cloud(self, points: np.ndarray) -> None: ...
    def show_trajectory_path(self, path: np.ndarray) -> None: ...
    def add_gui_controls(self, scenario: ValidationScenario) -> None: ...
```

#### Reporter (`reporter.py`)

```python
class ValidationReporter:
    def __init__(self, result: ValidationResult): ...

    def build(self, output_dir: Path) -> mjswanApp:
        """Uses mjswan Builder to produce a static site.
        The site includes:
        - Embedded MuJoCo simulation replay
        - Pass/fail badges per criterion
        - 3D view of trajectory + collision points
        - Summary dashboard (cycle time, reachability heatmap, etc.)
        """ ...
```

---

## 7. BOP (Bill of Process) Integration

### 7.1 What is Bill of Process (BOP)

In industrial manufacturing, a **Bill of Process (BOP)** defines the complete manufacturing job sequence — the ordered list of operations, workstations, tools, process parameters, and sequence dependencies required to produce an assembly. BOPs originate from PLM/MES systems (SAP, Siemens Teamcenter, Dassault Delmia, etc.) and are typically exchanged as XML, JSON, or STEP-NC.

### 7.2 BOP → Robot Command Pipeline

```
┌──────────────────────────────────────────────────────────────┐
│                  Bill of Process (BOP)                        │
│  From PLM/MES: SAP, Teamcenter, Delmia, etc.                 │
│  Format: XML / JSON / STEP-NC / CSV / Database               │
│                                                               │
│  [Op-10] Pick Part A from Tray-1                              │
│  [Op-20] Place Part A on Fixture-3                            │
│  [Op-30] Run Screw Fastening M6 × 4 (cross pattern)          │
│  [Op-40] Weld Joint-1 (path: J1 → J2 → J3)                  │
│  [Op-50] Inspect (vision checkpoint at viewpoint)             │
│  [Op-60] Place assembly on Outfeed Conveyor                  │
└──────────────────────┬───────────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│  BOP Parser & Operation Decomposition         │
│  (process/bop.py)                             │
│                                               │
│  For each operation:                          │
│  - Parse type, location refs, tool, params   │
│  - Resolve dependencies (before/after)       │
│  - Validate against known operation types    │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│  Location Resolver                            │
│  (process/location_resolver.py)               │
│                                               │
│  Map logical names → 3D poses in workcell:    │
│  "Tray-1"      → /workcell/table/tray1       │
│  "Fixture-3"   → /workcell/fixtures/fixture3 │
│  "Outfeed"     → /workcell/conveyor/outfeed  │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│  Motion Profile Generator                     │
│  (process/operation.py)                       │
│                                               │
│  Pick   → PTP(approach) → LIN(grasp) → LIN(retreat)            │
│  Place  → PTP(approach) → LIN(place) → LIN(retreat)            │
│  Screw  → PTP(align) → LIN(contact) → ScrewN(4×) → LIN(ret)   │
│  Weld   → PTP(start) → LIN(along path at weld_speed)           │
│  Inspect→ PTP(viewpoint) → Capture → PTP(next view)            │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│  Sequence Compiler                            │
│  (process/sequence_compiler.py)               │
│                                               │
│  Sorts by dependency DAG, flattens, applies  │
│  tool-change logic, produces ordered Task    │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│  Robot Command Sequence (ordered Task)        │
│                                               │
│  [ 1] PTP home → pre_grasp     q: [0,-45,90,...]              │
│  [ 2] LIN pre_grasp → grasp    t: (0.25, 0.50, 0.15)          │
│  [ 3] WAIT                      0.3s  (close gripper)         │
│  [ 4] LIN grasp → post_grasp   t: (0.25, 0.50, 0.35)          │
│  [ 5] PTP post_grasp → pre_place  q: [90,-15,30,...]          │
│  [ 6] LIN pre_place → place     t: (0.60, 0.00, 0.15)          │
│  [ 7] WAIT                      0.3s  (open gripper)           │
│  [ 8] LIN place → post_place    t: (0.60, 0.00, 0.35)          │
│  ...                                                           │
└──────────────────────────────────────────────────────────────┘
```

### 7.3 Module Interfaces

#### BOP Data Model (`process/bop.py`)

```python
@dataclass
class BOPOperation:
    id: str                           # e.g. "OP-10"
    type: OperationType               # PICK, PLACE, SCREW, WELD, INSPECT, DISPENSE, ...
    location_ref: str                 # Logical location name → resolved to 3D pose
    tool: str                         # e.g. "gripper_2f", "screwdriver_M6"
    params: dict[str, Any]            # Process parameters (torque, speed, count, pattern, ...)
    dependencies: list[str]           # Operation IDs that must precede this

@dataclass
class BillOfProcess:
    name: str
    operations: list[BOPOperation]
    sequences: list[SequenceConstraint]  # Ordering, parallelism constraints

class BOPParser:
    @staticmethod
    def from_xml(path: Path) -> BillOfProcess: ...
    @staticmethod
    def from_json(path: Path) -> BillOfProcess: ...
    @staticmethod
    def from_csv(path: Path) -> BillOfProcess: ...
    @staticmethod
    def from_step_nc(path: Path) -> BillOfProcess: ...
```

#### Location Resolver (`process/location_resolver.py`)

```python
class LocationResolver:
    """Map BOP logical location names → 3D poses in the workcell frame tree."""

    def __init__(self, workcell: Workcell): ...

    def register(self, name: str, frame_path: str, offset: np.ndarray | None = None):
        """e.g. register("Tray-1", "/workcell/table/tray1")"""

    def resolve(self, name: str) -> np.ndarray:
        """Return 4×4 pose in robot base frame."""
        ...
```

#### Operation Motion Profiles (`process/operation.py`)

```python
class OperationType(Enum):
    PICK = "PICK"
    PLACE = "PLACE"
    SCREW = "SCREW"
    WELD = "WELD"
    INSPECT = "INSPECT"
    DISPENSE = "DISPENSE"
    DEBURR = "DEBURR"
    PALETIZE = "PALLETIZE"

class MotionProfile:
    """Defines the robot motion pattern for each operation type."""

    @staticmethod
    def for_pick(location: np.ndarray, gripper_config) -> list[Command]:
        """PTP(approach) → LIN(contact) → WAIT(grip) → LIN(retract)"""

    @staticmethod
    def for_place(location: np.ndarray, gripper_config) -> list[Command]:
        """PTP(approach) → LIN(deposit) → WAIT(release) → LIN(retract)"""

    @staticmethod
    def for_screw(location: np.ndarray, tool_config) -> list[Command]:
        """PTP(align) → LIN(contact) → SCREW_N × count(torque-controlled) → LIN(retract)"""

    @staticmethod
    def for_weld(path: list[np.ndarray], weld_config) -> list[Command]:
        """PTP(start) → LIN(along path segments at weld_speed)"""

    @staticmethod
    def for_inspect(location: np.ndarray, camera_config) -> list[Command]:
        """PTP(viewpoint) → CAPTURE → PTP(next viewpoint)"""

    @staticmethod
    def for_dispense(location: np.ndarray, dispense_config) -> list[Command]:
        """PTP(start) → LIN(path at dispense_speed, tool ON) → tool OFF"""
```

#### Sequence Compiler (`process/sequence_compiler.py`)

```python
class SequenceCompiler:
    """BOP → ordered robot command sequence (Task)."""

    def __init__(
        self,
        location_resolver: LocationResolver,
        tool_registry: ToolRegistry,
        ik_solver: IKSolver,
        home_joints: np.ndarray,
    ): ...

    def compile(self, bop: BillOfProcess) -> Task:
        """Produce a flat, time-ordered robot command sequence.

        Steps:
        1. Resolve all location references to 3D poses
        2. Topological sort by dependency DAG
        3. For each operation, generate motion profile commands
        4. Insert tool-change commands where needed
        5. Flatten into a single ordered Task
        """
        ...

    def _topological_sort(self, operations: list[BOPOperation]) -> list[BOPOperation]:
        """Respects before/after constraints, detects cycles.""" ...
```

### 7.4 Complete Example: BOP → Validation

**Input BOP (XML):**
```xml
<BillOfProcess name="FrontLeftDoorAssembly">
  <Operation id="OP-10" type="PICK" location="Tray-A/side_brace" tool="gripper_2f">
    <Param name="approach_dist" value="100" unit="mm"/>
  </Operation>
  <Operation id="OP-20" type="PLACE" location="Fixture-3/locator_pin_1" tool="gripper_2f" after="OP-10"/>
  <Operation id="OP-30" type="SCREW" location="Fixture-3/screw_hole_1" tool="screwdriver_M6" after="OP-20">
    <Param name="torque" value="8" unit="Nm"/>
    <Param name="count" value="4"/>
    <Param name="pattern" value="cross"/>
  </Operation>
  <Operation id="OP-40" type="INSPECT" location="Fixture-3/viewpoint_1" tool="vision_cam" after="OP-30"/>
  <Operation id="OP-50" type="PLACE" location="Outfeed/QC_area" tool="gripper_2f" after="OP-40"/>
</BillOfProcess>
```

**Python API (full pipeline):**
```python
from robot_validator import (
    Robot, Workcell, ValidationScenario,
    CollisionCriterion, CycleTimeCriterion, JointLimitCriterion
)
from robot_validator.process import BOPParser, LocationResolver, SequenceCompiler
from robot_validator.controller import CCDIKSolver

# 1. Define workcell with location reference frames
workcell = (
    Workcell()
    .add_table(position=(0.5, 0, 0), size=(1.2, 0.8, 0.05))
    .add_frame("tray_a", parent="/workcell/table", position=(-0.3, 0.3, 0.05))
    .add_frame("fixture_3", parent="/workcell/table", position=(0.2, -0.2, 0.05))
    .add_frame("outfeed", position=(0.8, 0.5, 0))
    .add_conveyor("outfeed_belt", position=(0.8, 0.5, 0), size=(1.0, 0.3, 0.1))
)

# 2. Load robot
robot = Robot.from_urdf("panda.urdf")

# 3. Parse BOP from PLM export
bop = BOPParser.from_xml("front_left_door_assembly.xml")

# 4. Register BOP logical locations → workcell frames
resolver = LocationResolver(workcell)
resolver.register("Tray-A/side_brace", "/workcell/table/tray_a",
                   offset=[0, 0, 0.1])  # part ± 10cm offset
resolver.register("Fixture-3/locator_pin_1", "/workcell/table/fixture_3")
resolver.register("Fixture-3/screw_hole_1", "/workcell/table/fixture_3",
                   offset=[0.05, 0.05, 0.02])
resolver.register("Fixture-3/viewpoint_1", "/workcell/table/fixture_3",
                   offset=[0.3, 0, 0.3])

# 5. Compile BOP → robot command sequence
commands = SequenceCompiler(
    location_resolver=resolver,
    tool_registry=ToolRegistry()
        .register("gripper_2f", GripperConfig(width=0.08, force=50))
        .register("screwdriver_M6", ScrewdriverConfig(torque_max=12)),
    ik_solver=CCDIKSolver(),
    home_joints=[0, -45, 0, -90, 0, 45, 0],
).compile(bop)

# 6. Define validation criteria
criteria = [
    CollisionCriterion(pairs=["robot/*", "fixture/*"], severity="error"),
    CollisionCriterion(pairs=["robot/*", "robot/*"], severity="warning"),
    JointLimitCriterion(margin_deg=5.0, severity="warning"),
    CycleTimeCriterion(max_seconds=30.0, severity="error"),
    SafetyCriterion(zones="safety_cage", min_distance=0.05),
]

# 7. Run validation
scenario = ValidationScenario(
    name="front_left_door_assembly",
    robot=robot,
    workcell=workcell,
    task=commands,
    criteria=criteria,
)

result = scenario.validate()
print(result.summary())
#   Collision       : ✅ PASS (0 contacts)
#   Joint Limits    : ✅ PASS (all joints > 5° from limits)
#   Cycle Time      : ❌ FAIL (34.2s > 30.0s limit)
#     OP-30 SCREW   : 12.8s  (bottleneck — 4 screws × cross pattern)
#     OP-50 PLACE   : 5.5s
#   Safety Distance : ✅ PASS (min 0.12m > 0.05m)

# 8. Export report
scenario.report("validation_report_door_assembly/")
```

### 7.5 Supported BOP Formats

| Format | Common In | Parser Method | Phase |
|---|---|---|---|
| XML | Siemens Teamcenter, SAP PLM | `BOPParser.from_xml()` | 2A |
| JSON | Modern MES, custom tooling | `BOPParser.from_json()` | 2A |
| Dict/Programmatic | Direct Python usage | `BillOfProcess(operations=[...])` | 2A |
| STEP-NC (ISO 14649) | CNC-based manufacturing | `BOPParser.from_step_nc()` | 2B |
| CSV | Legacy systems, manual input | `BOPParser.from_csv()` | 2B |

### 7.6 Operation Type → Motion Profile Mapping

| Operation Type | Motion Profile | Key Parameters |
|---|---|---|
| `PICK` | Approach → Contact → Grip → Retract | approach_dist, gripper_width, grip_force |
| `PLACE` | Approach → Deposit → Release → Retract | approach_dist, release_delay |
| `SCREW` | Align → Contact → Screw N× → Retract | torque, rpm, count, pattern (cross/star/line) |
| `WELD` | Move to start → Weld path → Stop | weld_speed, voltage, wire_feed, path_points |
| `INSPECT` | Move to viewpoint → Capture → Next | viewpoints[], exposure, resolution |
| `DISPENSE` | Move to start → Dispense path → Stop | dispense_rate, bead_width, path |
| `DEBURR` | Move to start → Deburr path → Stop | feed_rate, tool_rpm, path |
| `PALLETIZE` | Pick → Place in grid pattern → Repeat | grid_rows, grid_cols, layer_height |

### 7.7 BOP-Specific Validation

In addition to general validation criteria, the BOP integration enables process-level checks:

```python
# Check if BOP is completable given workcell configuration
from robot_validator.validation import BOPCompletableCriterion

BOPCompletableCriterion(
    bop=bop,
    check_each_operation=(
        lambda op: True  # or custom per-operation checks
    )
)

# Check tool availability per operation
ToolAvailabilityCriterion(bop=bop, available_tools=["gripper_2f", "screwdriver_M6"])

# Check sequence dependency graph (no cycles, all refs resolvable)
BOPConsistencyCriterion(bop=bop)
```

---

## 8. Data Flow

### 8.1 Live Validation Session

```
User Input                    System                          Output
───────────                   ──────                          ──────
Load scenario YAML
       │
       ▼
                        scenario.py parses config
                        robot.py loads URDF → MuJoCo
                        workcell.py builds MuJoCo scene
       │
       ▼
                        visualizer.py starts viser server
                        Renders 3D scene via WebSocket
                        Adds GUI panels (sliders, buttons)
       │
       ▼
User adjusts params ──▶  viser events → update MuJoCo data
       │                 mj_step() physics tick
       │                 Criteria.check() run each tick
       │                 Results pushed to viser GUI
       │
       ▼
User clicks "Validate" ──▶  Full simulation run (task playback)
                            All criteria evaluated
                            Results shown in GUI + timeline
       │
       ▼
User clicks "Export"  ──▶  reporter.py builds static site
                            HTML report saved to output_dir
```

### 8.2 Headless / CI Validation

```
Scenario YAML / BOP XML
       │
       ▼
    cli.py validate --scenario scenario.yaml --bop assembly.xml --output report/
       │
       ▼
    session.py:
       ├── scenario.py.load()
       ├── robot.py.build()
       ├── workcell.py.build()
       ├── process/bop.py.parse()           # Parse BOP from PLM/MES
       ├── process/location_resolver.py     # Map logical names → 3D poses
       ├── process/sequence_compiler.py     # BOP → ordered robot commands
       ├── controller/*.step() loop (simulate full task)
       ├── validation/*.check() → accumulate results
       └── reporter.py.build() → static site
       │
       ▼
    report/index.html  (deployable to any static host)
```

---

## 9. Implementation Phases

### Phase 1: Foundation (Weeks 1–2)

| # | Task | Files | Description | Verification |
|---|---|---|---|---|
| 1.1 | Package scaffold | `robot_validator/__init__.py`, `pyproject.toml`, CLI scaffold | Hatchling build, Typer CLI skeleton | `pip install -e .` + `--help` |
| 1.2 | Robot model loader | `robot.py` | Load URDF via `yourdfpy`, convert to MuJoCo XML, introspect joints/links | Load a Panda URDF, print joint list |
| 1.3 | Workcell builder | `workcell.py` | Programmatic MuJoCo scene: boxes, cylinders, meshes, planes | Create table + fence, compile scene |
| 1.4 | Basic visualizer | `visualizer.py` | viser server: render robot in 3D, add joint sliders | Move sliders, robot follows |
| 1.5 | Integration test | `tests/` | End-to-end: load URDF → create workcell → launch viser | Green `pytest` |

**Phase 1 Deliverable:**
```
$ robot-validator serve examples/panda_pick.yaml
# Opens browser with interactive 3D view, joint sliders work
```

### Phase 2A: Core BOP & Validation (Week 3)

**Priority:** BOP XML/JSON parser only; defer CSV/STEP-NC to Phase 2B. IK solver promoted from Phase 3 to unblock reachability criterion.

| # | Task | Files | Description | Verification |
|---|---|---|---|---|
| 2.1 | IK solver | `controller/ik.py` | Damped least-squares (DLS) IK; MuJoCo Jacobian-based; CCD fallback for 6-DoF arms (DLS preferred for 7-DoF like Panda) | Reach a Cartesian target pose |
| 2.2 | BOP data model & parser | `process/bop.py` | `BillOfProcess`, `BOPOperation` dataclasses; `from_xml()`, `from_json()` parsers (CSV/STEP-NC deferred) | Parse a sample BOP XML → correct object tree |
| 2.3 | Location resolver | `process/location_resolver.py` | Register logical names → workcell frame paths; resolve to 4×4 poses | Resolve "Tray-1" → correct translation |
| 2.4 | Motion profiles | `process/operation.py` | `MotionProfile` with `for_pick()`, `for_place()`, `for_screw()`, `for_weld()`, `for_inspect()` | Generated commands match expected motion pattern |
| 2.5 | Sequence compiler | `process/sequence_compiler.py` | Topological sort, tool-change insertion, flatten to ordered `Task` | Compile multi-op BOP → correct command order |
| 2.6 | Base criterion | `validation/criteria.py` | `ValidationCriterion` ABC, `CriterionResult`, `CompositeCriterion` | Unit tests for AND/OR/NOT |
| 2.7 | Collision detection | `validation/collision.py` | Wrap `mj_collide()`, check contact pairs, distance to collision | Two intersecting boxes → fail |
| 2.8 | Kinematic limits | `validation/kinematics.py` | Joint limit margin, singularity proximity (Manipulability) | Pose nearing joint limit → warning |
| 2.9 | Cycle time | `validation/cycle_time.py` | Measure trajectory duration against max allowable | Trajectory → timed result |

**Phase 2A Deliverable:**
```
$ robot-validator validate examples/panda_pick.yaml --bop assembly.xml
Collision    : ✅ PASS  (0 contact pairs)
Joint Limits : ⚠️ WARNING (joint 4 at 87% of limit)
Cycle Time   : ❌ FAIL  (9.2s > 8.0s limit, OP-30 bottleneck at 12.8s)
```

### Phase 2B: Extended Validation & BOP (Week 4)

| # | Task | Files | Description | Verification |
|---|---|---|---|---|
| 2.10 | Reachability | `validation/reachability.py` | Sample target poses around workcell, solve IK → pass/fail heatmap (requires 2.1) | Check volume coverage |
| 2.11 | Safety distances | `validation/safety.py` | Minimum distance between robot and safety zones / humans | Zone intrusion → error |
| 2.12 | Workspace coverage | `validation/workspace.py` | Monte Carlo sampling, forward kinematics, AABB discretization | Verify reachable workspace volume |
| 2.13 | Extended BOP formats | `process/bop.py` | CSV, STEP-NC parsers (ISO 14649) — pluggable via `BOPParser.register_format()` | Parse additional BOP formats |
| 2.14 | BOP-specific validation | `process/` + `validation/` | `BOPCompletableCriterion`, `ToolAvailabilityCriterion`, `BOPConsistencyCriterion` | BOP with missing tool → fail |

**Phase 2B Deliverable:**
```
$ robot-validator validate examples/panda_pick.yaml --bop assembly.xml
Collision    : ✅ PASS  (0 contact pairs)
Reachability : ✅ PASS  (96.3% coverage)
Joint Limits : ⚠️ WARNING (joint 4 at 87% of limit)
Cycle Time   : ❌ FAIL  (9.2s > 8.0s limit, OP-30 bottleneck at 12.8s)
Safety       : ✅ PASS  (min 0.12m > 0.05m)
Workspace    : ✅ PASS  (89.1% coverage)
```

### Phase 3: Controllers & Trajectories (Weeks 5–6)

**Note:** IK solver moved to Phase 2A. This phase focuses on trajectory primitives and execution.

| # | Task | Files | Description | Verification |
|---|---|---|---|---|
| 3.1 | Base controller | `controller/base.py` | `Controller` ABC with `step()`, `reset()`, `is_done` | Unit tests |
| 3.2 | Joint position ctrl | `controller/joint_pos.py` | Smooth joint interpolation with velocity/accel limits | Move joint from A to B in 2s |
| 3.3 | PTP trajectory | `controller/trajectory.py` | Joint-space S-curve, trapezoidal velocity profile | Execute multi-point path |
| 3.4 | LIN trajectory | `controller/trajectory.py` | Cartesian linear with IK (from 2.1) at each step, look-ahead | Move EE in straight line |
| 3.5 | CIRC trajectory | `controller/trajectory.py` | 3-point arc, parameterize by angle | Trace a circle |
| 3.6 | Trajectory follower | `controller/trajectory_follower.py` | Step through trajectory waypoints on MuJoCo clock | Full pick-and-place playback |
| 3.7 | ONNX policy wrapper | `controller/onnx_policy.py` | Wrap ONNX model → `PolicyHandle` compatible with mjswan | Run a trained policy in loop |

**Phase 3 Deliverable:**
```
$ robot-validator serve examples/panda_pick.yaml
# "Run" button plays full pick-and-place trajectory in 3D
```

### Phase 4: Reporting & Static Deployment (Weeks 7–8)

| # | Task | Files | Description | Verification |
|---|---|---|---|---|
| 4.1 | Scenario persistence | `scenario.py` | `from_yaml()` / `to_yaml()`, schema validation | Round-trip serialization |
| 4.2 | Report builder | `reporter.py` | Use mjswan `Builder` to: embed MuJoCo XML, inject validation JSON, add UI | `build()` produces `index.html` |
| 4.3 | Report template | `reporter.py` | Custom mjswan scene: pass/fail badges, timeline chart, trajectory visualization | Open report in browser |
| 4.4 | Session orchestrator | `session.py` | High-level: `load → simulate → validate → build_report` | One-call validation pipeline |
| 4.5 | CLI polish | `cli.py` | `validate`, `serve`, `build`, `report` subcommands with rich output | Full CLI reference |
| 4.6 | CI integration docs | `docs/ci.md` | GitHub Actions example: validate on PR | CI green check |
| 4.7 | Batch validation | `session.py` | Run many scenarios, aggregate results | `--batch scenarios/*.yaml` |

**Phase 4 Deliverable:**
```
$ robot-validator build examples/assembly_scenario.yaml --bop examples/front_door_bop.xml -o report/
$ ls report/
  index.html         # Interactive validation report
  assets/            # Embedded MuJoCo WASM + models
  validation.json    # Structured data for external tooling
  bop_trace.json     # BOP operations → commands mapping with timing
```

---

## 10. Key Design Decisions

### 10.1 Simulate in mjswan, visualize in viser

| Concern | Decision | Rationale |
|---|---|---|
| Physics | mjswan (MuJoCo) | Accurate collision, dynamics, kinematics |
| Live interaction | viser (WebSocket) | Real-time bidirectional, rich GUI toolkit |
| Static reports | mjswan (template) | three.js + mujoco-wasm, deployable anywhere |

Communication boundary: `robot_validator` owns the MuJoCo `MjModel`/`MjData`. It steps physics with `mj_step()`, then pushes state to viser for rendering.

### 10.2 Validation Criteria Are Composable

```python
criteria = CompositeCriterion(
    name="pick_validation",
    operator="and",
    children=[
        CollisionCriterion(pairs=["robot/*", "fixture/*"]),
        ReachabilityCriterion(target_poses=poses),
        CycleTimeCriterion(max_seconds=8.0),
    ]
)
```

### 10.3 Two Deployment Modes

| Mode | Tech | Use Case |
|---|---|---|
| **Live session** | `viser serve` | Scenario authoring, debugging, interactive demos |
| **Static report** | `mjswan Builder.build()` (Python API) | CI/CD artifacts, stakeholder review, compliance audit |

### 10.4 Traditional Control via "Controller" Abstraction

mjswan is designed for ONNX policies. To support traditional industrial robots:

```
Controller (ABC)          ← joint_pos.py, trajectory_follower.py
    ↓ adapter
mjswan PolicyHandle       ← Reuses mjswan's scene integration pipeline
```

The `Controller.step()` returns joint position targets that get applied as MuJoCo actuators. This means traditional PTP/LIN control works even though mjswan expects an ONNX model.

### 10.5 URDF → MuJoCo Conversion Strategy

```
Robot URDF
    │
    ▼
yourdfpy (joints, links, geometry)
    │
    ├── Convert collision geometry → MuJoCo geoms
    ├── Convert visual geometry → Mesh assets
    ├── Map joints → MuJoCo joints (revolute, prismatic, fixed)
    └── Build actuator mapping (position control)
    │
    ▼
MuJoCo XML string → mujoco.MjModel
```

Use an in-memory `xml.etree.ElementTree` generation rather than file-based conversion. Cache the result for reuse.

---

## 11. Validation Criteria Catalog

### 11.1 Collision Checker

| Property | Detail |
|---|---|
| **Class** | `CollisionCriterion` |
**Input** | Contact pair patterns (e.g., `"robot/*"`, `"fixture/table"`) |
| **Check** | After simulation step, query `mjData.contact` for enabled pairs |
| **Output** | Pass if zero contacts; fail with list of colliding geoms + position |
| **Severity** | `error` |

Visual overlay: Red spheres at contact points in viser.

### 11.2 Reachability Checker

| Property | Detail |
|---|---|
| **Class** | `ReachabilityCriterion` |
| **Input** | Set of target poses (from CSV, grid, or interactively placed) |
| **Check** | For each pose, run IK. If IK converges within joint limits → reachable |
| **Output** | Pass if > threshold % reachable; fail with unreachable pose list |
| **Severity** | `error` or `warning` (configurable) |

Visual overlay: Green spheres (reachable) + Red spheres (unreachable) in viser.

### 11.3 Cycle Time Checker

| Property | Detail |
|---|---|
| **Class** | `CycleTimeCriterion` |
| **Input** | Full trajectory, max allowable time |
| **Check** | Simulate complete trajectory. Measure wall-clock from first to last step. |
| **Output** | Pass if `actual_time <= max_time`; fail with breakdown per segment |
| **Severity** | `error` |

### 11.4 Joint Limit Checker

| Property | Detail |
|---|---|
| **Class** | `JointLimitCriterion` |
| **Input** | Safety margin (degrees or percentage) |
| **Check** | At each step, check proximity of each joint to its limits |
| **Output** | Pass if all joints stay outside margin; warning/error per joint |
| **Severity** | `warning` (proximity) / `error` (at limit) |

### 11.5 Singularity Checker

| Property | Detail |
|---|---|
| **Class** | `KinematicCriterion` (singularity mode) |
| **Input** | Manipulability threshold |
| **Check** | Compute `sqrt(det(J @ J.T))` at each trajectory step |
| **Output** | Fail if manipulability drops below threshold; warning if approaching |
| **Severity** | `warning` → `error` |

### 11.6 Safety Distance Checker

| Property | Detail |
|---|---|
| **Class** | `SafetyCriterion` |
| **Input** | Safety zone geometry (cage, light curtain, human model) + min distance |
| **Check** | MuJoCo distance computation between robot geoms and zone geoms |
| **Output** | Fail if any distance < min_distance |
| **Severity** | `error` |

### 11.7 Workspace Coverage Checker

| Property | Detail |
|---|---|
| **Class** | `WorkspaceCriterion` |
| **Input** | Workspace bounding box (AABB), resolution, target volume percentage |
| **Check** | Monte Carlo sample joint configs, forward kinematics, discretize volume |
| **Output** | Pass if covered volume >= target percentage |
| **Severity** | `warning` |

---

## 12. Example Scenario Walkthrough

### 12.1 Scenario YAML

```yaml
name: "panda_pick_and_place_v2"
description: "Validate Panda robot picking parts from conveyor and placing on table"

# Option A: Inline task definition (as shown below)
# Option B: BOP file reference (uncomment to use)
# bop: "bop/pick_and_place_assembly.xml"
# location_mapping:
#   "Part/Tray-1": "/workcell/table/tray_a"
#   "Part/Fixture-3": "/workcell/table/fixture_3"

robot:
  urdf: "fr3.urdf"
  base_position: [0.0, 0.0, 0.0]
  end_effector: "panda_hand_tcp"
  joint_config:
    velocity_limits: [2.175, 2.175, 2.175, 2.175, 2.175, 2.175, 2.175]
    acceleration_limits: [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]

workcell:
  fixtures:
    - name: "table"
      type: "box"
      position: [0.6, 0.0, 0.0]
      dimensions: [1.0, 0.8, 0.05]
    - name: "conveyor"
      type: "box"
      position: [0.0, 0.5, 0.0]
      dimensions: [1.5, 0.3, 0.1]
  obstacles:
    - name: "safety_cage"
      type: "mesh"
      path: "cage.stl"
      position: [-0.3, 1.2, 0.0]
  safety_zones:
    - name: "restricted_area"
      type: "box"
      position: [0.0, 0.8, 0.5]
      dimensions: [0.5, 0.3, 1.0]

task:
  steps:
    - move: PTP
      target: [0.0, -30.0, 60.0, -90.0, 0.0, 45.0, 0.0]
      velocity: 1.0
    - move: LIN
      target_pos: [0.2, 0.5, 0.15]
      target_euler: [180, 0, 90]
      velocity: 0.3
    - wait: 0.5
    - move: LIN
      target_pos: [0.2, 0.5, 0.35]
      velocity: 0.3
    - move: PTP
      target: [90.0, -15.0, 30.0, -60.0, 45.0, 30.0, 0.0]
      velocity: 1.5
    - move: LIN
      target_pos: [0.6, 0.0, 0.15]
      velocity: 0.3
    - wait: 0.3
    - move: LIN
      target_pos: [0.6, 0.0, 0.35]
      velocity: 0.3

criteria:
  - collision:
      pairs: ["robot/*", "fixture/*", "robot/*", "obstacle/*"]
      severity: error
  - collision:
      pairs: ["robot/*", "safety/*"]
      severity: error
  - reachability:
      pose_source: "pick_poses.csv"
      threshold: 0.95
      severity: error
  - cycle_time:
      max_seconds: 8.0
      severity: error
  - joint_limits:
      margin_deg: 5.0
      severity: warning
  - singularity:
      manipulability_threshold: 0.01
      severity: warning
```

### 12.2 Python API Equivalent

```python
from robot_validator import (
    ValidationScenario, Robot, Workcell, Task,
    CollisionCriterion, ReachabilityCriterion,
    CycleTimeCriterion, JointLimitCriterion
)
from robot_validator.controller import PTP, LIN

scenario = ValidationScenario(
    name="panda_pick_and_place_v2",
    robot=Robot.from_urdf("fr3.urdf", base_position=(0, 0, 0)),
    workcell=(
        Workcell()
        .add_table(position=(0.6, 0, 0), size=(1.0, 0.8, 0.05))
        .add_conveyor(position=(0, 0.5, 0), size=(1.5, 0.3, 0.1))
        .add_obstacle_mesh("cage.stl", position=(-0.3, 1.2, 0))
        .add_safety_zone("restricted", position=(0, 0.8, 0.5), size=(0.5, 0.3, 1.0))
    ),
    task=(
        Task("pick_and_place")
        .add(PTP, target=[0, -30, 60, -90, 0, 45, 0], velocity=1.0)
        .add(LIN, target_pos=(0.2, 0.5, 0.15), velocity=0.3)
        .wait(0.5)
        .add(LIN, target_pos=(0.2, 0.5, 0.35), velocity=0.3)
        .add(PTP, target=[90, -15, 30, -60, 45, 30, 0], velocity=1.5)
        .add(LIN, target_pos=(0.6, 0, 0.15), velocity=0.3)
        .wait(0.3)
        .add(LIN, target_pos=(0.6, 0, 0.35), velocity=0.3)
    ),
    criteria=[
        CollisionCriterion(pairs=["robot/*", "fixture/*"], severity="error"),
        CollisionCriterion(pairs=["robot/*", "safety/*"], severity="error"),
        ReachabilityCriterion(pose_source="pick_poses.csv", threshold=0.95),
        CycleTimeCriterion(max_seconds=8.0),
        JointLimitCriterion(margin_deg=5.0, severity="warning"),
    ]
)

# Option A: Interactive
scenario.visualize().launch()

# Option B: Headless validation
result = scenario.validate()
print(result.summary())

# Option C: Export report
scenario.report("validation_report_panda_v2/")
```

---

## 13. Risks & Mitigations

| # | Risk | Impact | Likelihood | Mitigation |
|---|---|---|---|---|
| 1 | mjswan designed for RL policies, not traditional trajectory control | High | Medium | Build `controller/` layer with `Controller` ABC; wrap as mjswan-compatible `PolicyHandle` |
| 2 | MuJoCo wasm in browser may be too slow for complex workcells | Medium | Medium | Desktop MuJoCo for live sessions; wasm only for report replay (pre-recorded) |
| 3 | URDF → MuJoCo conversion has fidelity gaps (non-standard joints, complex meshes) | High | Low | Validate with `yourdfpy` test suite; support manual MuJoCo XML override |
| 4 | viser and mjswan template compete for 3D rendering | Low | Medium | Clear separation: viser for **live authoring**, mjswan for **deployed reports** |
| 5 | No existing IK in either framework | Medium | High | Implement damped least-squares (DLS) IK in `controller/ik.py`; CCD is weak for 7-DoF arms; MuJoCo's `mj_kinematics` provides Jacobian |
| 6 | Collision detection in MuJoCo requires correct geom pairing | Medium | Low | Provide sensible defaults (all robot geoms → all env geoms); allow user overrides |
| 7 | Validation results are only as good as the MuJoCo model fidelity | Medium | Low | Document model fidelity requirements; support importing from CAD (STL/URDF) |
| 8 | BOP format varies significantly across PLM vendors | Medium | High | Pluggable parser architecture; canonical `BillOfProcess` data model as intermediary; vendor-specific schema adapters |
| 9 | BOP logical location names may not match CAD/workcell frame names | Medium | High | Explicit `LocationResolver.register()` API; YAML mapping file fallback; runtime validation for unresolved refs |
| 10 | Two-server complexity (viser + mjswan serving) | Low | Medium | Phase 4 only; viser handles live, mjswan handles build; no simultaneous runtime needed |
| 11 | Phase 2 scope too ambitious (11 tasks in 2 weeks) | Medium | High | Split into 2A (core) and 2B (extended); defer non-critical items to later phases |
| 12 | Reachability criterion requires IK solver from Phase 3 (cross-phase dependency) | Medium | High | Move IK solver to start of Phase 2A; or defer reachability to Phase 3 |

---

## 14. Timeline & Milestones

```
Week 1   2   3   4   5   6   7   8
│   │   │   │   │   │   │   │   │
├─── Phase 1: Foundation ───┤
│   ██ Scaffold & Robot      │
│   ██ Workcell Builder     │
│   ██ Basic Visualizer     │
│                           │
        ├─── Phase 2A: Core ──┤  ├─── Phase 2B: Extended ──┤
        │   ██ IK + BOP XML     │  │   ██ Reachability       │
        │   ██ Collision        │  │   ██ Safety             │
        │   ██ Kinematics / Cycle│ │   ██ Workspace           │
        │   ████████████████████│  │   ██ Extended BOP formats│
        │                           │
                ├─── Phase 3: Controllers ───┤
                │   ██ PTP/LIN/CIRC           │
                │   ██ Trajectory Follower    │
                │   ██ ONNX Adapter           │
                │                               │
                        ├─── Phase 4: Reporting ───┤
                        │   ██ Scenario Persistence│
                        │   ██ Report Builder     │
                        │   ██ Session Orchestrator│
                        │   ██ CLI & CI           │
                        │                           │
                        ▼                           ▼
                   Milestone:                   Milestone:
                Interactive validation          CI-ready pipeline +
                with viser + MuJoCo             static HTML reports
```

### Milestone Descriptions

| Milestone | Week | Deliverable | Demo |
|---|---|---|---|
| M1 | 2 | Robot in browser | `robot-validator serve robot.urdf` → interactive 3D joint control |
| M2 | 4 | Validation engine | `robot-validator validate scenario.yaml` → pass/fail output |
| M3 | 6 | Trajectory control | Full pick-and-place playback with IK and collision checking |
| M4 | 8 | Complete system | CI pipeline + HTML reports + interactive sessions |

---

## 15. Appendix: Repo Deep-Dive

### 15.1 mjswan Source Map

```
src/mjswan/
├── __init__.py          # Version, exports
├── builder.py           # Builder class: fluent API entry point
├── app.py               # mjswanApp: launch built apps (static server)
├── project.py           # ProjectConfig, ProjectHandle
├── scene.py             # SceneConfig, SceneHandle
├── policy.py            # PolicyConfig, PolicyHandle
├── motion.py            # MotionConfig, MotionHandle
├── splat.py             # SplatConfig, SplatHandle
├── command.py           # UI command terms (slider, button, checkbox)
├── viewer_config.py     # Camera/viewer configuration
├── utils.py             # ZIP bundling, XML path rewriting, name2id
├── wandb_utils.py       # W&B artifact downloads for motions
├── _cli.py              # Typer CLI (build, serve, mjlab, etc.)
├── _build_client.py     # Frontend build orchestration (Vite/nodeenv)
├── adapters/            # mjlab compatibility layer
├── envs/mdp/            # MDP framework building blocks
│   ├── actions/         # Action terms (joint pos, muscle, etc.)
│   ├── events.py        # Event types
│   ├── observations.py  # Observation group configs
│   └── terminations.py  # Termination conditions
├── managers/            # RL loop managers
│   ├── action_manager.py
│   ├── event_manager.py
│   ├── observation_manager.py
│   └── termination_manager.py
└── template/            # Frontend (Vite + React + three.js + mujoco-wasm)
```

### 15.2 viser Source Map

```
src/viser/
├── __init__.py               # Version, exports
├── _viser.py                 # ViserServer: main server class
├── _scene_api.py             # Scene API: add_* method (mesh, frame, point cloud, etc.)
├── _scene_handles.py         # Scene node handles (properties, events)
├── _gui_api.py               # GUI API: add_* method (slider, button, text, etc.)
├── _gui_handles.py           # GUI handle classes (value changes, enable/disable)
├── _messages.py              # WebSocket message protocol
├── _assignable_props_api.py  # Property assignment helpers
├── _client_autobuild.py      # Client build entry point
├── transforms/               # SE(3) / SO(3) / SE(2) / SO(2) Lie groups
├── extras/                   # URDF, Colmap, Record3D
│   ├── _urdf.py              # ViserUrdf: URDF rendering helper
│   ├── _record3d.py          # Record3D stream support
│   └── colmap/               # COLMAP reconstruction support
├── theme/                    # UI theming
├── client/                   # Frontend (TypeScript)
└── infra/                    # Infrastructure (tunneling, etc.)
```

### 15.3 Key Dependencies Matrix

| Dependency | mjswan | viser | robot_validator |
|---|---|---|---|
| mujoco | 3.8.1 | — | via mjswan |
| onnx / onnxruntime | ✅ | — | optional |
| websockets | — | ✅ | via viser |
| numpy | ✅ | ✅ | ✅ |
| trimesh | — | ✅ | via viser |
| yourdfpy | — | optional | ✅ |
| nodeenv | ✅ | dev | — |
| three.js | via template | via client | via mjswan report |
| rich | ✅ | ✅ | ✅ |
| typer | ✅ | — | ✅ |
| msgspec | — | ✅ | via viser |

---

## 16. 2-Week MVP Plan

**Goal:** Demonstrate that the core concept works — load a robot into a workcell, execute a simple trajectory, detect collisions and joint limit violations, and visualize everything in real-time 3D.

### In Scope (MVP)

| # | Feature | Justification |
|---|---|---|
| 1 | Package scaffold + YAML scenario loader | Entry point; everything flows from a scenario config |
| 2 | Robot model loader (URDF → MuJoCo) | Core dependency — without a robot, nothing else matters |
| 3 | Workcell builder (fixtures, obstacles) | Needed for collision detection to have meaning |
| 4 | viser visualization server | The "wow factor" — interactive 3D demo in browser |
| 5 | Joint position controller + PTP trajectories | Simplest trajectory primitive; no IK needed; joint-space only |
| 6 | Collision detection criterion | The #1 thing users care about: will the robot crash? |
| 7 | Joint limit + singularity criteria | Quick wins; no external solver, just joint-space checks |
| 8 | Scenario orchestrator | Ties everything together: load → simulate → validate → visualize |

### Out of Scope (Deferred to Full Build)

| Feature | Deferral Rationale |
|---|---|
| BOP parsing (all formats) | Adds parser complexity; MVP uses inline YAML trajectories |
| IK solver | PTP trajectories operate in joint-space; no Cartesian targets needed |
| LIN / CIRC trajectories | Require IK; deferred to Phase 3 |
| Reachability / workspace coverage | Need IK + Monte Carlo; heavy for MVP |
| Cycle time criterion | Requires realistic velocity/accel profiles; nice-to-have |
| Safety distance criterion | Requires zone modeling; defer to Phase 2B |
| Static HTML reports (mjswan) | Frontend build pipeline is the largest integration surface |
| CLI polish + batch validation | MVP uses direct Python API |
| ONNX policy wrapper | RL policies are not the MVP audience |
| Tool registry / sequence compiler | BOP-dependent |

### Week-by-Week Schedule

#### Week 1: Core Infrastructure (Days 1–5)

| Day | Task | Deliverable |
|---|---|---|
| **1** | Package scaffold (`pyproject.toml`, `__init__.py`), YAML scenario loader (`scenario.py`) | `pip install -e .` works; `ValidationScenario.from_yaml()` parses → dataclass |
| **2** | Robot loader (`robot.py`): URDF → MuJoCo XML via `yourdfpy`, in-memory | Load Panda URDF → `MjModel`, print joint/geom names |
| **3** | Workcell builder (`workcell.py`): programmatic MuJoCo scene composition | Build table + obstacle → merge with robot model → valid `MjModel` |
| **4** | viser visualizer (`visualizer.py`): render MuJoCo robot + workcell, joint sliders in GUI | Browser: 3D scene renders; slider moves joint angle |
| **5** | Joint position controller (`controller/joint_pos.py`): smooth PTP interpolation with vel/accel limits, MuJoCo stepping loop | Move robot from home pose to target pose in simulation; joint positions follow trajectory |

**Week 1 Demo:**
```
Browser: 3D view of Panda robot above a table. Joint sliders move the arm.
Press "Run": robot executes PTP trajectory to a target pose.
```

#### Week 2: Validation + Integration (Days 6–10)

| Day | Task | Deliverable |
|---|---|---|
| **6** | Collision criterion (`validation/collision.py`): wrap `mj_collide()`, pattern-matched pair checking, red contact spheres in viser | Scenario with intentional collision → pass/fail shown in GUI |
| **7** | Joint limit criterion (`validation/kinematics.py`): margin check per joint per step | Pose near limit → warning badge in GUI; at limit → error |
| **8** | Scenario orchestrator (`session.py`): `load → build model → step simulation → run criteria → output results` | One function call: `session.run(scenario)` returns structured results |
| **9** | Integration scenario: full YAML config with PTP steps, collision test, joint limit test → visualize in viser with results panel | Demo scenario validates correctly: ✅/❌ per step, contact points highlighted |
| **10** | Polish, edge-case handling, documentation | README, example scenario, clean error messages |

**Week 2 Demo:**
```
$ python -m robot_validator.serve examples/mvp_panda.yaml
# Browser:
#   3D Scene: Panda robot, table, safety obstacle
#   GUI Panel: [Joint Sliders] [Run] [Reset]
#   Results Panel (after Run):
#     [1] PTP home → grasp       ✅ No collision, joint limits OK
#     [2] PTP grasp → place      ❌ Collision at t=0.6s (link3 ↔ obstacle)
#     [3] PTP place → retract    ✅ No collision, joint 4 at 88% limit ⚠️
#   3D Overlay: Red spheres at collision contact point on link3
```

### Deliverable

```
robot_validator/
├── src/robot_validator/
│   ├── __init__.py
│   ├── scenario.py              # ValidationScenario + YAML loader
│   ├── robot.py                 # URDF → MuJoCo conversion
│   ├── workcell.py              # Programmatic workcell builder
│   ├── visualizer.py            # viser integration
│   ├── session.py               # Orchestrator
│   ├── controller/
│   │   ├── __init__.py
│   │   ├── base.py              # Controller ABC
│   │   └── joint_pos.py         # PTP joint-space interpolation
│   └── validation/
│       ├── __init__.py
│       ├── criteria.py          # CriterionResult
│       ├── collision.py         # CollisionCriterion
│       └── kinematics.py        # JointLimitCriterion
├── examples/
│   └── mvp_panda.yaml           # Demo scenario
├── tests/
├── pyproject.toml
└── README.md
```

### Success Criteria

1. **Loads a robot:** URDF → MuJoCo model in <5s
2. **Displays in 3D:** Browser renders robot + workcell via viser, joint sliders work
3. **Executes trajectory:** PTP joint-space motion runs smoothly through MuJoCo physics
4. **Detects collisions:** Intentional collision → correct pass/fail, contact point visualized in 3D
5. **Checks joint limits:** Warning when approaching limit, error at limit
6. **One-command demo:** `python -m robot_validator.serve examples/mvp_panda.yaml` works end-to-end

### Risks Specific to MVP

| Risk | Mitigation |
|---|---|
| URDF → MuJoCo conversion is harder than expected | Start with one well-known URDF (Franka Emika Panda); don't generalize prematurely |
| viser integration with MuJoCo stepping is messy | Run MuJoCo in the main thread; push state snapshots to viser at 30 Hz |
| Collision detection requires correct geom naming/pairing conventions | Hard-code collision pairs for the demo; don't implement pattern matching until Day 7 |
| Day 9 integration reveals hidden dependencies | Build a minimal smoke test after Day 5 (just run, don't validate) to catch issues early |

### Post-MVP: What Comes Next

After 2 weeks, the MVP proves the architecture works. The remaining 6 weeks of the full plan then:
- **Weeks 3–4:** BOP parser, IK solver, reachability/cycle time/safety criteria
- **Weeks 5–6:** LIN/CIRC trajectories, full trajectory follower
- **Weeks 7–8:** Static reports via mjswan, CLI, batch validation, CI integration

*End of proposal.*
