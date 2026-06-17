"""viser integration — render MuJoCo robot + workcell in 3D, GUI controls."""

from __future__ import annotations

import mujoco
import numpy as np
import trimesh

import viser


class ValidationVisualizer:
    def __init__(self, model, data):
        self._model = model
        self._data = data
        self._server = None
        self._meshes = []
        self._sliders = {}

    def _trimesh_geom(self, i):
        g = self._model.geom(i)
        sz = self._model.geom_size[i]
        if g.type == mujoco.mjtGeom.mjGEOM_BOX:
            return trimesh.creation.box(extents=2 * sz)
        if g.type == mujoco.mjtGeom.mjGEOM_CAPSULE:
            return trimesh.creation.capsule(radius=sz[0], height=sz[1])
        if g.type == mujoco.mjtGeom.mjGEOM_SPHERE:
            return trimesh.creation.icosphere(radius=sz[0])
        return None

    def _add_meshes(self):
        scene = self._server.scene
        self._meshes = []
        for i in range(self._model.ngeom):
            tm = self._trimesh_geom(i)
            if tm is not None:
                pos = self._data.geom_xpos[i]  # numpy array
                quat = self._matrix_to_wxyz(self._data.geom_xmat[i].reshape(3, 3))
                h = scene.add_mesh_trimesh(f"/geoms/{i}", mesh=tm, position=pos, wxyz=quat)
                self._meshes.append(h)

    @staticmethod
    def _matrix_to_wxyz(m):
        """Convert 3×3 rotation matrix to (w, x, y, z) quaternion."""
        trace = m[0, 0] + m[1, 1] + m[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (m[2, 1] - m[1, 2]) * s
            y = (m[0, 2] - m[2, 0]) * s
            z = (m[1, 0] - m[0, 1]) * s
        else:
            if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
                w = (m[2, 1] - m[1, 2]) / s
                x = 0.25 * s
                y = (m[0, 1] + m[1, 0]) / s
                z = (m[0, 2] + m[2, 0]) / s
            elif m[1, 1] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
                w = (m[0, 2] - m[2, 0]) / s
                x = (m[0, 1] + m[1, 0]) / s
                y = 0.25 * s
                z = (m[1, 2] + m[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
                w = (m[1, 0] - m[0, 1]) / s
                x = (m[0, 2] + m[2, 0]) / s
                y = (m[1, 2] + m[2, 1]) / s
                z = 0.25 * s
        return np.array([w, x, y, z], dtype=np.float64)

    def _add_sliders(self):
        gui = self._server.gui
        for i in range(self._model.njnt):
            j = self._model.joint(i)
            # Only add sliders for revolute joints (type=3), not free/fixed
            if j.type[0] not in (0, 4, 5):  # 0=free, 4=ball, 5=slider
                lo, hi = float(j.range[0]), float(j.range[1])
                qidx = int(j.qposadr[0])
                initial = float(self._data.qpos[qidx])
                # Clamp initial to be within range
                initial = max(lo, min(hi, initial))
                s = gui.add_slider(label=j.name, min=lo, max=hi, step=0.1, initial_value=initial)
                self._sliders[j.name] = (qidx, s)

    def launch(self, host="0.0.0.0", port=8080):
        self._server = viser.ViserServer(host=host, port=port)
        self._server.scene.add_grid(
            "ground",
            width=20.0,
            height=20.0,
            plane="xy",  # horizontal floor (Z is up in this MuJoCo model)
            plane_color=(217, 217, 217),
            plane_opacity=0.5,
        )
        # Compute forward kinematics so geom_xpos reflects joint positions
        mujoco.mj_forward(self._model, self._data)
        self._add_meshes()
        self._add_sliders()
        print(f"Viser at {host}:{port}")
        self._server.sleep_forever()
