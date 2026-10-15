"""Initial orbit for independent task-box bodies; no arm/controller changes."""
from pathlib import Path
import xml.etree.ElementTree as ET


def initialize_free_plugs(scene, model_path: Path, orbital_position, bus_velocity):
    names = set(scene.getBodyNames())
    selected = names.intersection(('guide_plug', 'aviation_plug', 'cylindrical_plug', 'bolt', 'fastener'))
    if not selected:
        return
    root = ET.parse(model_path).getroot()
    for name in selected:
        if name == 'fastener':
            captive = root.find('.//body[@name="locking_rod"]/body[@name="fastener"]')
            if (captive is not None and captive.find('joint[@name="fastener_slide"]') is not None
                    and captive.find('joint[@name="fastener_spin"]') is not None
                    and root.find('./equality/joint[@name="fastener_helix"]') is not None):
                # Its parent already inherits the orbital transform/velocity.
                continue
        element = root.find(f'./worldbody/body[@name="{name}"]')
        if element is None or (element.find('freejoint') is None and element.find('joint[@type="free"]') is None):
            raise ValueError(f'{name} must be an independent world body')
        local = [float(v) for v in element.get('pos', '0 0 0').split()]
        body = scene.getBody(name)
        body.setPosition([float(a) + b for a, b in zip(orbital_position, local, strict=True)])
        body.setVelocity([float(v) for v in bus_velocity])
