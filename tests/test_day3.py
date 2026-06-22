"""Day 3 smoke test — workcell builder merges with robot model."""

import sys

sys.path.insert(0, "/home/z0023d3d/project/robotSim/src")

import mujoco

from robot_validator.robot import Robot
from robot_validator.workcell import WorkcellBuilder

robot = Robot.from_mjcf("examples/models/franka_fr3/fr3.xml")
wb = WorkcellBuilder()
wb.add_box("table", (0.6, 0.0, 0.0), (1.0, 0.8, 0.05))
final = wb.merge(robot.xml)
model = mujoco.MjModel.from_xml_string(final)
print(f"Joints: {model.njnt}")
print(f"Bodies: {model.nbody}")
print(f"Geoms: {model.ngeom}")
print("Day 3 workcell builder: merged robot + table ✓")
