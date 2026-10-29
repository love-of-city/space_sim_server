"""Validated fixed-layout inputs for the web collection launcher."""
from pathlib import Path
import xml.etree.ElementTree as ET

from .collection_layouts import digest


def requires_color_layout(task):
    from simulation.scripted_tasks.randomization import COLOR_RANDOMIZATION_TASKS
    return task in COLOR_RANDOMIZATION_TASKS


def fixed_layout_input(model, catalog):
    """Validate real geometry/material bindings without changing any source file."""
    if not model or not catalog:
        raise ValueError("固定采集布局必须同时配置 model 和 catalog")
    from simulation.task_box_layout import validated_capture_layout
    from simulation.socket_colors import apply_socket_colors, sample_color_assignment

    model, catalog = Path(model).expanduser().resolve(), Path(catalog).expanduser().resolve()
    layout = validated_capture_layout(model, catalog)
    if layout.get("schema") not in ("task-box-layout/2", "task-box-layout/3"):
        raise ValueError("固定采集布局需要具有独立孔位的操作台模型")
    # Exercise the same seven-hole binding validation used by episode writing,
    # on a private XML tree. No variant, native scene or UE process is created.
    apply_socket_colors(ET.parse(model).getroot(), sample_color_assignment(seed=0))
    record = dict(schema="collection-fixed-layout/1", model=str(model), catalog=str(catalog),
                  model_sha256=digest(model), catalog_sha256=digest(catalog),
                  manifest_sha256=digest(model.with_suffix(".layout.json")),
                  attitude_control_sha256=digest(model.with_name("attitude_control.json")))
    return record, layout


def validate_fixed_snapshot(record):
    current, layout = fixed_layout_input(record["model"], record["catalog"])
    if current != record:
        raise ValueError("固定采集模型或 catalog 在批次创建后发生变化，请重新创建批次")
    return layout


def validate_saved_layout(scene, record):
    """Never silently relabel a saved scene as a different configured layout."""
    layout = scene.get("task_box_layout") or {}
    selected = (scene.get("collection_config") or {}).get("layout_randomization") or {}
    if scene.get("collection_layout_randomization") and not layout and not selected.get("enabled"):
        raise ValueError("已保存布局场景缺少可核对的模型指纹，不能替换为默认固定布局")
    if layout and layout.get("model_sha256") != record["model_sha256"]:
        raise ValueError("已保存场景的布局与固定采集模型不一致")
    if selected.get("enabled") and (
            selected.get("model_sha256") != record["model_sha256"]
            or selected.get("catalog_sha256") != record["catalog_sha256"]):
        raise ValueError("已保存场景的模型或 catalog 与固定采集配置不一致")
