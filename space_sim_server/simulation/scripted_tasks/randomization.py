"""Episode sampling and immutable model variants for scripted collection."""

from copy import deepcopy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from space_arm_platform.collection_randomization import CollectionRandomization


def generator(seed, name, tick=0):
    # Stable named streams: adding/toggling another option never moves this stream.
    salt = int.from_bytes(hashlib.sha256(name.encode()).digest()[:8], "little")
    return np.random.Generator(
        np.random.PCG64(np.random.SeedSequence([int(seed), salt, int(tick)]))
    )


def options_for_scene(scene):
    cfg = scene.get("collection_config", {})
    if "randomization_options" in cfg:
        return CollectionRandomization.model_validate(cfg["randomization_options"])
    if "collection_randomization" in scene:
        return CollectionRandomization.model_validate(scene["collection_randomization"])
    if (
        any(cfg.get("disturbances", {}).values())
        or cfg.get("lighting_randomized")
        or cfg.get("dynamics_randomized")
    ):
        raise ValueError("Enabled legacy flags require explicit randomization_options")
    return CollectionRandomization()


def prepare_episode(scene):
    """Called once per attempted episode, before the recorder snapshots metadata."""
    options = options_for_scene(scene)
    seed = int(scene.get("seed", 0))
    realized = {}
    previous = scene.get("collection_config", {}).get("realized_randomization", {})
    if not options.lighting.enabled and "base_sunlight_intensity_scale" in previous:
        scene.setdefault("environment", {}).setdefault("lighting", {})["sunlight_intensity_scale"] = previous["base_sunlight_intensity_scale"]
    if options.joint_bias.enabled:
        span = options.joint_bias.max_abs_deg
        realized["joint_bias_deg"] = (
            generator(seed, "joint_bias").uniform(-span, span, 6).tolist()
        )
    for key, amplitude in (
        ("mass_scale", options.dynamics.mass_variation_pct),
        ("friction_scale", options.dynamics.friction_variation_pct),
    ):
        realized[key] = (
            float(
                generator(seed, key).uniform(1 - amplitude / 100, 1 + amplitude / 100)
            )
            if options.dynamics.enabled
            else 1.0
        )
    if options.lighting.enabled:
        light = scene.setdefault("environment", {}).setdefault("lighting", {})
        # Preserve base lighting so repeating preparation is idempotent.
        previous = scene.get("collection_config", {}).get("realized_randomization", {})
        base = previous.get(
            "base_sunlight_intensity_scale", light.get("sunlight_intensity_scale", 1.0)
        )
        span = options.lighting.variation_pct / 100
        factor = float(generator(seed, "lighting").uniform(1 - span, 1 + span))
        light["sunlight_intensity_scale"] = float(base) * factor
        realized.update(
            base_sunlight_intensity_scale=base,
            sunlight_factor=factor,
            sunlight_intensity_scale=light["sunlight_intensity_scale"],
        )
    cfg = scene.setdefault("collection_config", {})
    cfg.update(
        randomization_schema="collection-randomization/1",
        randomization_options=options.model_dump(),
        realized_randomization=realized,
        random_streams="PCG64/SeedSequence(seed,sha256(name)[:8],tick)",
        disturbances={
            k: v
            for k, v in options.flags().items()
            if k not in ("lighting", "dynamics")
        },
        lighting_randomized=options.lighting.enabled,
        dynamics_randomized=options.dynamics.enabled,
        observation_delay_scope="joint_proprioception_only",
        safety_and_task_feedback="ground_truth",
        action_recording="applied_joint_reference",
    )
    return scene


def write_model_variant(source, output, scene):
    """Scale one standalone MJCF consistently for BOTH MJScene and local MuJoCo.

    Uniform mass scaling also scales inertia, preserving centers of mass. Original
    assets are referenced by absolute path; source files are never modified.
    """
    source = Path(source).resolve()
    dynamics_enabled = options_for_scene(scene).dynamics.enabled
    task = scene.get('scripted_task_name', scene.get('collection_config', {}).get('task'))
    threaded_task = task in {'aviation_bolt_insert', 'locking_rod_cycle'}
    if not dynamics_enabled and not threaded_task:
        return source
    realized = scene.get("collection_config", {}).get("realized_randomization", {})
    mass, friction = (realized["mass_scale"], realized["friction_scale"]) if dynamics_enabled else (1., 1.)
    tree = ET.parse(source)
    root = tree.getroot()
    initialization = {}
    if task == 'aviation_bolt_insert' and root.find('.//geom[@name="bolt_storage_thread_sdf"]') is None:
        from tools.bolt_storage_thread import apply_bolt_storage_thread
        initialization['bolt_storage_thread'] = apply_bolt_storage_thread(root)
    if task == 'locking_rod_cycle':
        from tools.threaded_task_initialization import apply_locking_cycle_initial_state
        initialization['locking_rod'] = apply_locking_cycle_initial_state(root)
    if root.find(".//include") is not None:
        raise ValueError(
            "Dynamics randomization currently requires a standalone MJCF (no include)"
        )
    compiler = root.find("compiler")
    assetdir = compiler.get("assetdir", "") if compiler is not None else ""
    meshdir = compiler.get("meshdir", assetdir) if compiler is not None else assetdir
    texturedir = (
        compiler.get("texturedir", assetdir) if compiler is not None else assetdir
    )
    for item in root.findall("./asset/*"):
        for key in (
            "file",
            "fileup",
            "filedown",
            "fileleft",
            "fileright",
            "filefront",
            "fileback",
        ):
            file = item.get(key)
            if file:
                directory = (
                    meshdir
                    if item.tag == "mesh"
                    else texturedir
                    if item.tag == "texture"
                    else assetdir
                )
                item.set(key, (source.parent / directory / file).resolve().as_posix())
    if compiler is not None:
        for key in ("meshdir", "texturedir", "assetdir"):
            compiler.attrib.pop(key, None)
    default = root.find("default")
    if default is None:
        default = ET.SubElement(root, "default")
    geom = default.find("geom")
    if geom is None:
        geom = ET.SubElement(default, "geom")
    # Explicit root defaults cover geoms that previously inherited MuJoCo defaults.
    if "density" not in geom.attrib:
        geom.set("density", "1000")
    if "friction" not in geom.attrib:
        geom.set("friction", "1 0.005 0.0001")

    def scale(item, key, factor):
        if key in item.attrib:
            item.set(
                key,
                " ".join(
                    format(float(x) * factor, ".17g") for x in item.get(key).split()
                ),
            )

    for item in root.iter():
        if item.tag == "inertial":
            for key in ("mass", "diaginertia", "fullinertia"):
                scale(item, key, mass)
        elif item.tag == "geom":
            for key in ("mass", "density"):
                scale(item, key, mass)
            scale(item, "friction", friction)
        elif item.tag == "pair":
            # Explicit pair friction overrides geom friction. Fill its implicit default.
            if "friction" not in item.attrib:
                item.set("friction", "1 1 0.005 0.0001 0.0001")
            scale(item, "friction", friction)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    settings = source.with_name("attitude_control.json")
    if settings.is_file():
        from shutil import copyfile

        copyfile(settings, output.with_name("attitude_control.json"))
    scene.setdefault("collection_config", {})["model_randomization"] = dict(
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        variant_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        mass_scale=mass,
        friction_scale=friction,
        scope="all_bodies_and_contact_geoms",
        task_initialization=initialization,
    )
    return output
