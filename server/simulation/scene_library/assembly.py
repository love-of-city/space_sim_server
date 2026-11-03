"""Build inspectable mounted target fixtures from the delivered satellite.

These start already rigidly connected and are expressly not docking evidence.
The original model and its task hardware are never overwritten or scaled.
"""
from copy import deepcopy
from pathlib import Path
import math
import shutil
import xml.etree.ElementTree as ET

from .manifest import digest, model_dependencies, write_json
from .scaling import scale_satellite


def _absolutize(root, source):
    compiler = root.find("compiler")
    settings = {} if compiler is None else dict(compiler.attrib)
    for node in root.findall("./asset/*"):
        directory = settings.get("meshdir" if node.tag == "mesh" else "texturedir", settings.get("assetdir", ""))
        for key, value in list(node.attrib.items()):
            if key == "file" or key.startswith("file") and key[4:] in ("up", "down", "left", "right", "front", "back"):
                node.set(key, (source.parent / directory / value).resolve().as_posix())
    if compiler is not None:
        for key in ("meshdir", "texturedir", "assetdir"):
            compiler.attrib.pop(key, None)


def mounted_target_fixture(source, output, *, scale=1.5, deck_raise_m=.45, cross_transfer=False,
                           arm_mount_offset_m=None, receiver_shift_xy_m=(0., 0.)):
    """Generate the second-family post-dock development fixture.

    A rigid parent models an initially locked pair. The existing independent
    capture_target remains a separately identified, parked source asset. The new
    docked_target is a clone, with its own names and scaled collision meshes.
    """
    if not .25 <= deck_raise_m <= .8:
        raise ValueError("Deck elevation must be in [.25,.8] m")
    arm_mount_offset = list(arm_mount_offset_m if arm_mount_offset_m is not None
                            else (-.1, 0., .08 if cross_transfer else .25))
    if (len(arm_mount_offset) != 3 or not all(math.isfinite(v) for v in arm_mount_offset)
            or not 0. <= arm_mount_offset[2] <= .6):
        raise ValueError("Arm mount offset requires three finite values and height in [0,.6] m")
    receiver_shift_xy_m = list(receiver_shift_xy_m)
    if len(receiver_shift_xy_m) != 2 or not all(math.isfinite(v) and abs(v) <= .5 for v in receiver_shift_xy_m):
        raise ValueError("Receiver shift requires two finite offsets within +/-0.5 m")
    source, output = Path(source).resolve(), Path(output).resolve()
    dependencies = model_dependencies(source)
    original = ET.parse(source).getroot()
    scaled, scaling = scale_satellite(original, "capture_target", scale)
    root = deepcopy(original)
    root.find('./worldbody/body[@name="capture_target"]').set("pos", "5 0 0")
    assets = root.find("asset")
    scaled_assets = {n.get("name"): n for n in scaled.findall("./asset/mesh")}
    for name in scaling["cloned_meshes"].values():
        assets.append(deepcopy(scaled_assets[name]))
    shell = deepcopy(scaled.find('./worldbody/body[@name="capture_target"]'))
    names = {n.get("name"): "mounted_" + n.get("name") for n in shell.iter() if n.get("name")}
    for node in shell.iter():
        for key, value in list(node.attrib.items()):
            if key == "name" or key in ("joint", "joint1", "joint2", "body", "body1", "body2", "site", "site1", "site2"):
                if value in names:
                    node.set(key, names[value])
    for node in list(shell):
        if node.tag == "freejoint" or node.tag == "joint" and node.get("type") == "free":
            shell.remove(node)
    # Satellite broad face is rotated into the work-deck plane. Hardware keeps
    # its delivered dimensions and orientation, above that satellite face.
    # Delivered central bus bounds, measured from compiled visual meshes without
    # the solar-wing children. Do not mistake wing span for a mounting surface.
    central_bus_bounds = [[.0424999958, -.0737500003, -.3000786995],
                          [.3646814851, .0832499998, .3000787003]]
    mounting_face_xy = [(central_bus_bounds[1][i] - central_bus_bounds[0][i]) * scale for i in (0, 2)]
    if mounting_face_xy[0] < .4661 or mounting_face_xy[1] < .5661:
        raise ValueError("Scaled central bus face cannot contain the delivered task box with 5 mm edge margin")
    center_x = .5 * (central_bus_bounds[0][0] + central_bus_bounds[1][0])
    shell_position = [.35 - center_x * scale, .34,
                      deck_raise_m - .125 - central_bus_bounds[1][1] * scale]
    shell.set("pos", " ".join(map(str, shell_position)))
    shell.set("quat", "0.7071067811865476 0.7071067811865476 0 0")
    folded_panel = shell.find('.//body[@name="mounted_satellite_outer_panel"]')
    if folded_panel is not None:
        # Fold the wing away from the service-arm base. Keep its passive hinge;
        # the authored zero pose is changed in this variant, not during a run.
        panel_position = [float(v) for v in folded_panel.get("pos", "0 0 0").split()]
        panel_position[1] -= .015 * scale
        folded_panel.set("pos", " ".join(map(str, panel_position)))
        folded_panel.set("quat", "0 0 0 1")
    service = root.find('./worldbody/body[@name="cubesat_bus"]')
    arm = service.find('body[@name="link1"]')
    if arm is not None:
        position = [float(v) for v in arm.get("pos", "0 0 0").split()]
        position = [v + offset for v, offset in zip(position, arm_mount_offset)]
        arm.set("pos", " ".join(map(str, position)))
        # base_link_geom includes the spacecraft hull, not just an arm flange.
        # Preserve all source hull geometry. A separate adapter supports link1.
        if arm_mount_offset[2] > 0:
            half_height = arm_mount_offset[2] / 2
            ET.SubElement(service, "geom", name="service_arm_mount_pedestal", type="box",
                          size=f".065 .065 {half_height}",
                          pos=f"{arm_mount_offset[0]} {arm_mount_offset[1]} {half_height}",
                          mass=str(5. * arm_mount_offset[2]),
                          rgba=".35 .4 .45 1", contype="1", conaffinity="1", group="2")
    host = ET.SubElement(service, "body", name="docked_target", pos="0 0 0")
    host.append(shell)
    for index, (x, y) in enumerate(((.14, .18), (.56, .18), (.14, .58), (.56, .58))):
        ET.SubElement(host, "geom", name=f"target_deck_standoff_{index}",
                      type="cylinder", size=".008 .045", pos=f"{x} {y} {deck_raise_m-.085}",
                      mass=".05", rgba=".35 .4 .45 1", contype="1", conaffinity="1", group="2")
    moved = []
    for name in ("task_box_base", "drawer_base"):
        body = service.find(f'body[@name="{name}"]')
        if body is None:
            raise ValueError(f"Missing delivered module: {name}")
        position = [float(v) for v in body.get("pos", "0 0 0").split()]
        position[2] += deck_raise_m
        body.set("pos", " ".join(map(str, position)))
        service.remove(body)
        host.append(body)
        moved.append(name)
    for name in ("aviation_plug", "guide_plug", "cylindrical_plug", "bolt"):
        body = root.find(f'./worldbody/body[@name="{name}"]')
        position = [float(v) for v in body.get("pos", "0 0 0").split()]
        position[2] += deck_raise_m
        body.set("pos", " ".join(map(str, position)))
    receiver_modules = []
    receiver_world_offset = None
    if cross_transfer:
        # The delivered receiver board and precision bores move together. Their
        # source transforms are retained; no holes or fit tolerances are altered.
        task_box = host.find('body[@name="task_box_base"]')
        receiver_world_offset = [-.655 + receiver_shift_xy_m[0], receiver_shift_xy_m[1], .15 - deck_raise_m]
        offset = [float(v) + delta for v, delta in zip(task_box.get("pos").split(), receiver_world_offset)]
        receiver = ET.SubElement(service, "body", name="service_receiver_mount",
                                 pos=" ".join(map(str, offset)))
        for name in ("asm_board1", "board1_plug_bore_colliders"):
            node = task_box.find(f'body[@name="{name}"]')
            if node is None:
                raise ValueError(f"Missing paired receiver geometry {name}")
            task_box.remove(node)
            receiver.append(node)
            receiver_modules.append(name)
        bolt = root.find('./worldbody/body[@name="bolt"]')
        bolt.set("pos", " ".join(str(float(v) + delta) for v, delta in zip(bolt.get("pos").split(), receiver_world_offset)))
        for index, (x, y) in enumerate(((-.25, .20), (-.07, .20), (-.25, .50), (-.07, .50))):
            local = [x+receiver_shift_xy_m[0]-offset[0], y+receiver_shift_xy_m[1]-offset[1], .10-offset[2]]
            ET.SubElement(receiver, "geom", name=f"service_receiver_standoff_{index}",
                          type="cylinder", size=".006 .05", pos=" ".join(map(str, local)),
                          mass=".03", rgba=".35 .4 .45 1", contype="1", conaffinity="1", group="2")
    _absolutize(root, source)
    output.mkdir(parents=True, exist_ok=False)
    model = output / "mounted-target.xml"
    ET.indent(root)
    ET.ElementTree(root).write(model, encoding="utf-8", xml_declaration=True)
    shutil.copyfile(source.with_name("attitude_control.json"), output / "attitude_control.json")
    manifest = dict(schema="space-arm-mounted-target/1", model=str(model), sha256=digest(model),
                    source_dependencies=dependencies, scaling=scaling,
                    deck_raise_m=deck_raise_m, transferred_modules=moved,
                    receiver_modules=receiver_modules,
                    receiver_world_offset_m=receiver_world_offset,
                    receiver_shift_xy_m=receiver_shift_xy_m if cross_transfer else None,
                    central_bus_bounds_source_m=central_bus_bounds,
                    shell_position_m=shell_position, mounting_standoffs=4,
                    arm_mount_offset_m=arm_mount_offset if arm is not None else None,
                    source_base_link_geometry_preserved=True,
                    arm_mount_pedestal_mass_kg=5. * arm_mount_offset[2] if arm is not None else 0.,
                    mounting_face_xy_m=mounting_face_xy,
                    mounting_face_calibration="delivered_capture_target_central_body_visual_bounds_not_solar_wings",
                    wing_variant=dict(outer_panel_fold_deg=180, hinge_clearance_offset_source_m=.015,
                                      passive_hinge_preserved=True),
                    intended_family=4 if cross_transfer else 2,
                    target_body="docked_target", source_target_body="capture_target",
                    initial_condition="rigidly_predocked_fixture", physical_docking_accepted=False,
                    assembly_status="candidate", task_accepted=False,
                    required_checks=["native_model_compile", "mount_geometry", "initial_contacts",
                                     "full_arm_reachability", "native_task", "physical_docking_handoff"],
                    limitations=["No docking maneuver has been performed", "Original free target retained as parked asset",
                                 "Mounting support and sweep clearance still require verification",
                                 "Drawer remains a slide mechanism; detachable drawer transfer is not implemented"])
    write_json(output / "assembly.json", manifest)
    # Keep the parked free source target out of the worksite in native reset too.
    from .manifest import initial_scene_for_model
    initial = initial_scene_for_model(model)
    initial["randomization"]["target_position_m"] = [5., 0., 0.]
    initial["randomization"]["target_linear_velocity_m_s"] = [0., 0., 0.]
    write_json(output / "initial-scene.json", initial)
    return manifest
