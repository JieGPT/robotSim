"""Day 3 smoke test — workcell builder merges with robot model."""
import sys
sys.path.insert(0, '/home/z0023d3d/project/robotSim/src')

from robot_validator.robot import Robot, _build_mjcf
from robot_validator.workcell import WorkcellBuilder
from yourdfpy import URDF
import mujoco

urdf = URDF.load('examples/models/fr3_panda.urdf')
xml = _build_mjcf(urdf, (0, 0, 0))
wb = WorkcellBuilder()
wb.add_box('table', (0.6, 0.0, 0.0), (1.0, 0.8, 0.05))
final = wb.merge(xml)
model = mujoco.MjModel.from_xml_string(final)
print(f'Joints: {model.njnt}')
print(f'Bodies: {model.nbody}')
print(f'Geoms: {model.ngeom}')
print('Day 3 workcell builder: merged robot + table ✓')