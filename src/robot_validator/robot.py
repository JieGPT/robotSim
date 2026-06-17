"""URDF loader — convert to MuJoCo MjModel."""
from __future__ import annotations
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from yourdfpy import URDF

def _pos(origin):
    if origin is None:
        return "0 0 0"
    p = origin[:3, 3]
    return f"{p[0]} {p[1]} {p[2]}"

def _quat(origin):
    if origin is None:
        return "0 0 0 1"
    q = Rotation.from_matrix(origin[:3, :3]).as_quat()
    return f"{q[0]} {q[1]} {q[2]} {q[3]}"

class Robot:
    """Load a URDF, convert to MuJoCo MjModel, expose joint info."""
    def __init__(self, model, data, urdf):
        self.model = model
        self.data = data
        self.urdf = urdf

    @property
    def joint_names(self):
        return [self.model.joint(i).name for i in range(self.model.njnt)]

    @property
    def joint_limits(self):
        return np.array(self.model.jnt_range)

    @property
    def n_actuated_joints(self):
        return self.model.nu

    @property
    def actuator_names(self):
        return [self.model.actuator(i).name for i in range(self.model.nu)]

    @property
    def xml(self):
        return _build_mjcf(self.urdf, (0, 0, 0))

    @property
    def qpos_home(self):
        return np.zeros(self.model.nu, dtype=np.float64)

    @classmethod
    def from_urdf(cls, path, base_position=(0, 0, 0)):
        urdf = URDF.load(path)
        xml = _build_mjcf(urdf, base_position)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        return cls(model, data, urdf)

__all__ = ["Robot"]

def _build_mjcf(urdf, base_pos=(0, 0, 0)):
    root = ET.Element("mujoco")
    root.set("model", "robot")
    compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    default = ET.SubElement(root, "default")
    defl_j = ET.SubElement(default, "joint")
    defl_j.set("damping", "0.1")
    defl_j.set("frictionloss", "0.01")
    defl_g = ET.SubElement(default, "geom")
    defl_g.set("condim", "1")
    wb = ET.SubElement(root, "worldbody")
    base = ET.SubElement(wb, "body")
    base.set("name", "base")
    base.set("pos", f"{base_pos[0]} {base_pos[1]} {base_pos[2]}")
    kids = _link_children(urdf)
    _walk(base, urdf.base_link, None, urdf, kids)
    act = ET.SubElement(root, "actuator")
    for jn in urdf.actuated_joint_names:
        motor = ET.SubElement(act, "position")
        motor.set("joint", jn)
    return ET.tostring(root, encoding="unicode")

def _link_children(urdf):
    out = {}
    for jn, jo in urdf.joint_map.items():
        p = getattr(jo, "parent", None)
        c = getattr(jo, "child", None)
        if p and c:
            out.setdefault(p, []).append((jn, c))
    return out

def _walk(parent_body, link_name, in_joint, urdf, children):
    link = urdf.link_map[link_name]
    if in_joint:
        jo = urdf.joint_map[in_joint]
        t = getattr(jo, "type", "fixed")
        if t != "fixed":
            ax = list(jo.axis) if hasattr(jo, "axis") and jo.axis is not None else [0, 0, 1]
            jnt = ET.SubElement(parent_body, "joint")
            jnt.set("name", jo.name)
            if t == "revolute":
                jnt.set("type", "hinge")
                jnt.set("axis", f"{ax[0]} {ax[1]} {ax[2]}")
                lim = getattr(jo, "limit", None)
                jnt.set("limited", "true" if lim and hasattr(lim, "lower") else "false")
                if lim and hasattr(lim, "lower"):
                    jnt.set("range", f"{lim.lower} {lim.upper}")
            elif t == "continuous":
                jnt.set("type", "hinge")
                jnt.set("axis", f"{ax[0]} {ax[1]} {ax[2]}")
                jnt.set("limited", "false")
            elif t == "prismatic":
                jnt.set("type", "slide")
                jnt.set("axis", f"{ax[0]} {ax[1]} {ax[2]}")
                lim = getattr(jo, "limit", None)
                jnt.set("limited", "true" if lim and hasattr(lim, "lower") else "false")
                if lim and hasattr(lim, "lower"):
                    jnt.set("range", f"{lim.lower} {lim.upper}")

    if hasattr(link, "collisions") and link.collisions:
        for i, c in enumerate(link.collisions):
            g = ET.SubElement(parent_body, "geom")
            g.set("name", f"{link_name}_col_{i}")
            g.set("size", "0.03 0.1")
            g.set("contype", "1")
            g.set("conaffinity", "1")
            g.set("rgba", "0 0 0 0")
            orig = getattr(c, "origin", None)
            if orig is not None:
                g.set("pos", _pos(orig))
    else:
        g = ET.SubElement(parent_body, "geom")
        g.set("name", f"{link_name}_col")
        g.set("type", "capsule")
        g.set("size", "0.03 0.1")
        g.set("contype", "1")
        g.set("conaffinity", "1")
        g.set("rgba", "0 0 0 0")

    if hasattr(link, "inertial") and link.inertial is not None and link.inertial.mass > 0:
        ie = ET.SubElement(parent_body, "inertial")
        ie.set("mass", f"{link.inertial.mass:.4f}")
        if link.inertial.origin is not None:
            ie.set("pos", _pos(link.inertial.origin))
        if link.inertial.inertia is not None:
            i = link.inertial.inertia
            ie.set("diaginertia", f"{i[0,0]:.6f} {i[1,1]:.6f} {i[2,2]:.6f}")

    for cjin, clink in children.get(link_name, []):
        child = ET.SubElement(parent_body, "body")
        child.set("name", clink)
        cjo = urdf.joint_map[cjin]
        if cjo.origin is not None:
            child.set("pos", _pos(cjo.origin))
            child.set("quat", _quat(cjo.origin))
        _walk(child, clink, cjin, urdf, children)