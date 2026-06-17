# Workcell builder — add env objects to robot model
from __future__ import annotations
import xml.etree.ElementTree as ET
import mujoco
from robot_validator.scenario import WorkcellConfig

class WorkcellBuilder:
    def __init__(self):
        self._objects = []
    def add_box(self, name, pos, size):
        self._objects.append({"name": name, "type": "box", "pos": pos, "size": size})
        return self
    def add_mesh(self, name, path, pos):
        self._objects.append({"name": name, "type": "mesh", "path": path, "pos": pos})
        return self
    def add_plane(self, name, pos, normal):
        self._objects.append({"name": name, "type": "plane", "pos": pos, "normal": normal})
        return self
    def build(xml_string):
        root = ET.fromstring(xml_string)
        wb = root.find(".//worldbody")
        if wb is None:
            wb = ET.SubElement(root, "worldbody")
        return ET.tostring(root, encoding="unicode")
    def merge(self, xml_string):
        root = ET.fromstring(xml_string)
        wb = root.find(".//worldbody")
        if wb is None:
            wb = ET.SubElement(root, "worldbody")
        for obj in self._objects:
            b = ET.SubElement(wb, "body")
            b.set("name", obj["name"])
            b.set("pos", f"{obj['pos'][0]} {obj['pos'][1]} {obj['pos'][2]}")
            t = ET.SubElement(b, "geom")
            t.set("name", obj["name"])
            if obj["type"] == "box":
                t.set("type", "box")
                t.set("size", f"{obj['size'][0]/2:.3f} {obj['size'][1]/2:.3f} {obj['size'][2]/2:.3f}")
            elif obj["type"] == "plane":
                t.set("type", "plane")
                n = obj.get("normal", [0, 1, 0])
                t.set("size", f"{n[0]} {n[1]} {n[2]}")
            else:
                t.set("type", "capsule")
                t.set("size", "0.03 0.1")
            t.set("contype", "1")
            t.set("conaffinity", "1")
        return ET.tostring(root, encoding="unicode")

def from_config(xml_string, config: WorkcellConfig):
    builder = WorkcellBuilder()
    for f in (config.fixtures or []):
        builder.add_box(f["name"], f["position"], f["dimensions"])
    for o in (config.obstacles or []):
        builder.add_box(o["name"], o["position"], o["dimensions"])
    return builder.merge(xml_string)