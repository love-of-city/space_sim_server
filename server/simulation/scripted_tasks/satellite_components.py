"""Independent copies of named satellite assemblies, using existing mesh assets.

Positions and centers are in capture_target coordinates, in metres. Masses are
simulation assumptions, with uniform bounding-box inertia (not CAD measurements).
Collision meshes use the convex hulls of solid parts, excluding decorative faces.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class SatelliteComponent:
    assembly_id: str
    geoms: tuple[str, ...]
    collision_geoms: tuple[str, ...]
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    mass_kg: float
    collision_primitives: tuple[tuple[str, tuple, tuple], ...] = ()

    def snapshot(self):
        definition = json.loads(json.dumps(asdict(self)))
        if not self.collision_primitives:
            definition.pop("collision_primitives")
        return dict(schema="satellite-component/1", source_body="capture_target",
                    inertia_model="uniform-bounding-box", mass_source="simulation-assumption",
                    collision_model=("component-primitives" if self.collision_primitives else "solid-part-convex-hulls"),
                    collision_class="object_geom", **definition)


SATELLITE_COMPONENTS = {
    "sun_sensor": SatelliteComponent(
        "instance_0075",
        ("instance_0077_part_034_color_00", "instance_0078_part_035_color_00",
         *(f"instance_0079_part_036_color_{i:02d}" for i in range(6))),
        ("instance_0077_part_034_color_00", "instance_0079_part_036_color_00"),
        (.2454, -.064, -.16767129987001067), (.0465, .0195, .032), .08),
    "gnss_antenna": SatelliteComponent(
        "instance_0036",
        ("instance_0037_part_020_color_00", "instance_0037_part_020_color_01",
         "instance_0038_part_021_color_00", "instance_0038_part_021_color_01",
         *(f"instance_{i:04d}_part_022_color_00" for i in range(39, 43))),
        ("instance_0037_part_020_color_00", "instance_0038_part_021_color_01",
         *(f"instance_{i:04d}_part_022_color_00" for i in range(39, 43))),
        (.24314902279420017, -.00925, -.18117129987001068),
        (.07999492854519735, .081, .042), .25),
    "data_antenna": SatelliteComponent(
        "instance_0081", ("instance_0081_part_037_color_00",),
        (),
        (.11235, -.00525, .2672150279148613), (.09, .09, .016842655569744014), .15,
        # Source OBJ planar faces: z=.00211367221513 and .00612967221513.
        # A whole-mesh hull incorrectly fills air from the rear connector to
        # the plate edge, creating a wedge that ejects the object when pinched.
        (("box", (0., 0., .00412167221513), (.045, .045, .002008)),
         # Rear shaft and collar are separate; a radius-6.8 mm envelope would
         # inflate the actual radius-3.175 mm shaft at the bottom of the part.
         ("cylinder", (0., 0., -.00365382778487), (.003175, .0047675)),
         ("cylinder", (0., 0., -.00035382778487), (.00635, .0014675)),
         *(("cylinder", (x, y, -.000854633656905), (.0024, .002968305872035))
           for x, y in ((-.0044, -.0044), (-.0044, .0044), (.0044, -.0044), (.0044, .0044))),
         *(("cylinder", (x, y, .0072755), (.0025, .001146)) for x, y in
           ((-.041, -.022), (-.041, .022), (.041, -.022), (.041, .022),
            (-.022, -.041), (-.022, .041), (.022, -.041), (.022, .041),
            (-.032, 0.), (.032, 0.), (0., -.032), (0., .032))))),
}


def build_satellite_component(root, component_id, body_name):
    """Copy direct visual members only; leave the original satellite intact."""
    spec = SATELLITE_COMPONENTS[component_id]
    source = root.find('.//body[@name="capture_target"]')
    if source is None:
        raise ValueError("Satellite component requires capture_target assembly")
    body = ET.Element("body", name=body_name)
    sx, sy, sz = spec.size_m
    inertia = [spec.mass_kg * v / 12 for v in (sy*sy+sz*sz, sx*sx+sz*sz, sx*sx+sy*sy)]
    ET.SubElement(body, "inertial", pos="0 0 0", mass=str(spec.mass_kg),
                  diaginertia=" ".join(format(v, ".17g") for v in inertia))
    for name in spec.geoms:
        original = source.find(f'geom[@name="target_{name}"]')
        if original is None or original.get("type") != "mesh":
            raise ValueError(f"Missing satellite component mesh: {name}")
        visual = deepcopy(original)
        visual.set("name", f"{body_name}_{name}")
        pos = [float(v) for v in original.get("pos", "0 0 0").split()]
        visual.set("pos", " ".join(format(v-c, ".17g") for v, c in zip(pos, spec.center_m)))
        visual.attrib.update(mass="0", contype="0", conaffinity="0", group="2")
        body.append(visual)
        if name in spec.collision_geoms:
            # New geom: never inherit display-only overrides such as contype=0.
            attrs = {key: visual.get(key) for key in ("type", "mesh", "pos", "quat") if key in visual.attrib}
            collider = ET.SubElement(body, "geom", **attrs)
            collider.attrib.update(name=f"{body_name}_{name}_collision",
                                   **{"class": "object_geom"}, mass="0", contype="1",
                                   conaffinity="1", group="3", rgba="0 0 0 0")
    for index, (kind, pos, size) in enumerate(spec.collision_primitives):
        collider = ET.SubElement(body, "geom", name=f"{body_name}_collision_{index}",
                                 type=kind, pos=" ".join(map(str, pos)), size=" ".join(map(str, size)))
        collider.attrib.update(**{"class": "object_geom"}, mass="0", contype="1",
                               conaffinity="1", group="3", rgba="0 0 0 0")
    ET.SubElement(body, "freejoint", name=f"{body_name}_free")
    for item in body.iter():
        tags = ("joint", "freejoint") if item.tag == "freejoint" else (item.tag,)
        if "name" in item.attrib and any(root.find(f'.//{tag}[@name="{item.get("name")}"]') is not None for tag in tags):
            raise ValueError("Satellite component instance names must be unique")
    return body
