"""Episode sampling and immutable model variants for scripted collection."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from space_arm_platform.collection_randomization import CollectionRandomization


COLOR_RANDOMIZATION_SCHEMA = "socket-color-randomization/1"
COLOR_RANDOMIZATION_TASKS = frozenset({
    "plug_insert",
    "aviation_plug_insert",
    "guide_plug_insert",
    "cylindrical_plug_insert",
    "aviation_bolt_insert",
})
# Every scripted task that can use an independent task-box layout receives the
# same appearance mapping. Only plug/bolt tasks put colour words in the prompt.
COLOR_APPEARANCE_TASKS = COLOR_RANDOMIZATION_TASKS | frozenset({
    "drawer_cycle", "drawer_lift", "locking_rod_cycle",
})


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


def _assignment_digest(hole_colors, source_colors):
    """Match the digest used by ``simulation.socket_colors``.

    Keeping the digest in the scene before a model is compiled lets the
    recorder identify the intended appearance even when startup fails. The
    renderer applies the same assignment and writes the resulting digest back
    after validating the MJCF.
    """
    payload = {"holes": hole_colors, "sources": source_colors}
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def _colour_prompt_values(task, hole_colors, source_colors, palette, target_site=None):
    """Resolve instruction colours using the physical source/target identities."""
    from simulation.scripted_tasks.plug_profiles import is_plug_task, plug_profile

    def label(key):
        data = palette[key]
        return data.get("color_name", data.get("colour_name", key))

    if is_plug_task(task):
        profile = plug_profile(task)
        target = target_site or profile.target_site
        expected = {
            profile.target_site,
            profile.prefix + "_insert_40p1",
        }
        if target not in expected:
            raise ValueError(f"Target {target!r} is not this plug's assembly hole")
        # ``plug_insert`` is the compatibility alias for cylindrical_plug_insert.
        source = profile.body
        target_key = hole_colors[target]
        source_key = source_colors[source]
        return dict(source_color=label(source_key), target_color=label(target_key))
    if task == "aviation_bolt_insert":
        if target_site is not None and target_site != "bolt_assembly_hole":
            raise ValueError(f"Target {target_site!r} is not this bolt's assembly hole")
        source_key = source_colors["bolt"]
        target_key = hole_colors["bolt_assembly_hole"]
        return dict(source_color=label(source_key), target_color=label(target_key))
    raise ValueError(f"Colour prompt is unsupported for task {task!r}")


def _prepare_color_randomization(scene, cfg):
    """Attach one immutable appearance assignment to eligible episodes."""
    task = scene.get("scripted_task_name", cfg.get("task"))
    if task not in COLOR_APPEARANCE_TASKS:
        return None
    # A batch planner may provide an assignment. Preserve it exactly and only
    # derive the prompt/digest fields below.
    # The canonical assignment remains authoritative on recorder re-entry;
    # the color_randomization mapping is a derived metadata wrapper whose
    # schema may differ from the assignment schema.
    existing = cfg.get("episode_color_assignment") or cfg.get("color_randomization")
    if existing is not None:
        # Existing assignments may come from a batch planner. Recompute only
        # derived fields, preserving its sampled hole/source colours exactly.
        holes = existing.get("hole_colors", existing.get("holes"))
        sources = existing.get("source_colors", existing.get("sources"))
        if not isinstance(holes, dict) or not isinstance(sources, dict):
            raise ValueError("color_randomization requires hole_colors/source_colors")
        assignment = dict(existing)
    else:
        from simulation.socket_colors import sample_color_assignment

        seed = int(scene.get("seed", 0))
        # Sample the two groups from independent named streams. A change to
        # source parts therefore cannot perturb the hole permutation.
        assignment = sample_color_assignment(
            hole_rng=generator(seed, "hole-colors-v1"),
            source_rng=generator(seed, "source-colors-v1"),
        )
        assignment.update(seed=seed, randomized=True)
    from simulation.socket_colors import (
        ASSIGNMENT_SCHEMA, COLOR_PALETTE, normalize_color_assignment,
    )
    assignment = normalize_color_assignment(assignment)
    assignment_schema = assignment.get("schema", ASSIGNMENT_SCHEMA)

    holes = dict(assignment.get("hole_colors", assignment.get("holes", {})))
    sources = dict(assignment.get("source_colors", assignment.get("sources", {})))
    # Validate all expected identities before recording a prompt. Full MJCF
    # validation happens again in apply_socket_colors during model writing.
    from simulation.socket_colors import TARGET_SOCKET_IDS, SOURCE_IDS
    missing = [key for key in TARGET_SOCKET_IDS if key not in holes]
    missing += [key for key in SOURCE_IDS if key not in sources]
    if missing:
        raise ValueError(f"color_randomization is missing identities: {missing}")
    digest = _assignment_digest(holes, sources)
    from .instructions import STREAM, prepare_instruction
    values = None
    if task in COLOR_RANDOMIZATION_TASKS:
        values = _colour_prompt_values(
            task, holes, sources, COLOR_PALETTE,
            cfg.get("target_site") or scene.get("target_site"),
        )
    seed = int(scene.get("seed", 0))
    instruction = prepare_instruction(cfg, task, seed, generator(seed, STREAM), values)
    prompt = instruction["instruction"] if values is not None else None
    previous_wrapper = cfg.get("color_randomization")
    applied_schema = (
        previous_wrapper.get("applied_schema")
        if isinstance(previous_wrapper, dict)
        else None
    )
    cfg["color_randomization"] = {
        **assignment,
        "schema": COLOR_RANDOMIZATION_SCHEMA,
        "assignment_schema": assignment_schema,
        "seed": int(assignment.get("seed", scene.get("seed", 0))),
        "color_stream": "socket-colors-v1",
        "hole_stream": "hole-colors-v1",
        "source_stream": "source-colors-v1",
        "palette": deepcopy(COLOR_PALETTE),
        "hole_colors": holes,
        "source_colors": sources,
        "bolt_storage_color": assignment["bolt_storage_color"],
        "assignment_sha256": digest,
        "appearance_sha256": digest,
        **({"applied_schema": applied_schema} if applied_schema is not None else {}),
        **({"prompt_instruction": prompt} if prompt is not None else {}),
    }
    return cfg["color_randomization"]


def prepare_episode(scene):
    """Called once per attempted episode, before the recorder snapshots metadata."""
    options = options_for_scene(scene)
    seed = int(scene.get("seed", 0))
    realized = {}
    cfg = scene.setdefault("collection_config", {})
    previous = cfg.get("realized_randomization", {})

    # Every attempted episode carries one complete seven-colour assignment,
    # even when its task does not need a colour prompt. The two groups use
    # independent named streams and are preserved on recorder replay.
    from simulation.socket_colors import normalize_color_assignment, sample_color_assignment
    assignment = (
        cfg.get("episode_color_assignment")
        or cfg.get("color_randomization")
        or previous.get("episode_color_assignment")
    )
    if assignment is None:
        assignment = sample_color_assignment(
            hole_rng=generator(seed, "hole-colors-v1"),
            source_rng=generator(seed, "source-colors-v1"),
        )
        assignment["seed"] = seed
        assignment["stream"] = "socket-colors-v1"
    else:
        assignment = normalize_color_assignment(assignment)
        assignment.setdefault("seed", seed)
    cfg["episode_color_assignment"] = assignment
    realized["episode_color_assignment"] = deepcopy(assignment)

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
    _prepare_color_randomization(scene, cfg)
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
    config = scene.setdefault("collection_config", {})
    # Keep the canonical episode assignment separate from the prompt/palette
    # metadata wrapper. The MJCF applier records the assignment schema, while
    # the wrapper carries the collection-level schema and instruction text.
    color_assignment = config.get("episode_color_assignment") or config.get("color_randomization")
    color_config = config.get("color_randomization") or color_assignment
    visual_color = False
    probe_tree = None
    if color_config is not None:
        # Generated independent layouts have separately colourable mounting
        # surfaces. The default model shares one board mesh across several
        # holes, so recolouring the board cannot realize a seven-hole mapping.
        socket_body_names = (
            "aviation_plug", "guide_plug", "cylindrical_plug", "bolt",
            "layout_aviation_pickup", "layout_guide_pickup", "layout_cylindrical_pickup",
            "layout_aviation_insert_40p1", "layout_aviation_insert_40p5",
            "layout_guide_insert_40p1", "layout_guide_insert_40p5",
            "layout_cylindrical_insert_40p1", "layout_cylindrical_insert_40p5",
            "layout_bolt_assembly_hole", "layout_bolt_storage_hole",
        )
        probe_tree = ET.parse(source)
        body_names = {body.get("name") for body in probe_tree.findall(".//body")}
        missing = [name for name in socket_body_names if name not in body_names]
        visual_color = not missing
        if not visual_color and task in COLOR_RANDOMIZATION_TASKS:
            raise ValueError(
                "Colour-instruction tasks require an independent task-box layout "
                "with separately colourable holes; the model is missing bindings: "
                + ", ".join(missing)
            )
    tree = probe_tree if probe_tree is not None else ET.parse(source)
    root = tree.getroot()
    # Physical storage retention belongs to the platform, even when another
    # object is the task target. Use this same preparation for preflight and
    # recording; a source layout predating the liner is never run unchanged.
    from simulation.model_preparation.bolt_storage_thread import ensure_bolt_storage_thread
    storage_thread = ensure_bolt_storage_thread(root)
    physics_upgraded = storage_thread is not None and storage_thread['status'] == 'installed'
    if storage_thread is not None:
        config['model_physics'] = dict(
            schema='collection-model-physics/1', source_model=str(source),
            source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            bolt_storage_thread=storage_thread)
    if not dynamics_enabled and not threaded_task and not visual_color and not physics_upgraded:
        return source
    realized = scene.get("collection_config", {}).get("realized_randomization", {})
    mass, friction = (realized["mass_scale"], realized["friction_scale"]) if dynamics_enabled else (1., 1.)
    initialization = {}
    if storage_thread is not None:
        initialization['bolt_storage_thread'] = storage_thread
    appearance = None
    if visual_color:
        from simulation.socket_colors import apply_socket_colors

        appearance_assignment = dict(color_assignment)
        appearance_assignment["schema"] = color_config.get(
            "assignment_schema", color_config.get("schema")
        )
        appearance = apply_socket_colors(root, appearance_assignment)
    if task == 'locking_rod_cycle':
        from simulation.model_preparation.threaded_task_initialization import apply_locking_cycle_initial_state
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
    # A colour-only variant must not introduce inherited physical defaults.
    # Dynamics/threaded variants retain the historical explicit defaults.
    if dynamics_enabled or threaded_task:
        default = root.find("default")
        if default is None:
            default = ET.SubElement(root, "default")
        geom = default.find("geom")
        if geom is None:
            geom = ET.SubElement(default, "geom")
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

    if dynamics_enabled or threaded_task:
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
    model_randomization = dict(
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        variant_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        mass_scale=mass,
        friction_scale=friction,
        scope=("all_bodies_and_contact_geoms" if (dynamics_enabled or threaded_task)
               else "appearance_and_storage_thread_upgrade" if visual_color and physics_upgraded
               else "storage_thread_upgrade" if physics_upgraded
               else "appearance_only_existing_visible_geoms"),
        task_initialization=initialization,
    )
    if visual_color:
        # ``apply_socket_colors`` normalizes aliases and reports the exact
        # assignment that reached the material layer. Keep the scene prompt
        # and the rendered model tied to this digest.
        color_config["appearance_sha256"] = appearance["assignment_sha256"]
        color_config["applied_schema"] = appearance["schema"]
        visual_record = dict(
            source_sha256=model_randomization["source_sha256"],
            variant_sha256=model_randomization["variant_sha256"],
            assignment_sha256=appearance["assignment_sha256"],
            assignment_schema=appearance["assignment_schema"],
            scope="visible socket and plug materials only; physical preparation recorded in model_physics",
        )
        config["model_visual_randomization"] = visual_record
        model_randomization["visual_randomization"] = visual_record
    config["model_randomization"] = model_randomization
    if storage_thread is not None:
        config['model_physics']['variant_sha256'] = model_randomization['variant_sha256']
    return output
