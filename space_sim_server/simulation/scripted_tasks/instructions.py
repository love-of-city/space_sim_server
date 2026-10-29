"""Versioned English instruction templates and episode-scoped selection."""
from functools import lru_cache
import hashlib
import json
from pathlib import Path
from string import Formatter

BANK_PATH = Path(__file__).resolve().parents[2] / "config/task-instructions.json"
BANK_SCHEMA = "task-instructions/1"
SELECTION_SCHEMA = "task-instruction-selection/1"
STREAM = "task-instruction-v1"
TASKS = ("drawer_cycle", "aviation_plug_insert", "guide_plug_insert",
         "cylindrical_plug_insert", "aviation_bolt_insert", "locking_rod_cycle")
COLOR_TASKS = frozenset(TASKS[1:5])


def canonical_task(task):
    return "cylindrical_plug_insert" if task == "plug_insert" else task


def validate_template(task, template):
    if not isinstance(template, str) or not template.strip() or not template.isascii():
        raise ValueError("Instruction template must be nonempty English text")
    fields = set()
    for _, field, spec, conversion in Formatter().parse(template):
        if field is not None:
            if spec or conversion:
                raise ValueError("Instruction placeholders cannot use format specifications")
            fields.add(field)
    expected = {"source_color", "target_color"} if task in COLOR_TASKS else set()
    if fields != expected:
        raise ValueError(f"Incorrect instruction placeholders for {task}")


@lru_cache(maxsize=1)
def load_bank():
    bank = json.loads(BANK_PATH.read_text(encoding="utf-8"))
    if bank.get("schema") != BANK_SCHEMA or set(bank.get("tasks", {})) != set(TASKS):
        raise ValueError("Unsupported instruction bank schema or task list")
    for task, templates in bank["tasks"].items():
        if not isinstance(templates, list) or len(templates) != 20:
            raise ValueError(f"Task {task} requires exactly 20 instruction templates")
        for template in templates:
            validate_template(task, template)
        if len(set(templates)) != 20:
            raise ValueError(f"Task {task} has duplicate instruction templates")
    return bank


def prepare_instruction(cfg, task, seed, rng, values=None):
    """Freeze a template once; render colours from the current authoritative assignment.

    Save the actual template as well as its index so replay survives bank edits.
    A new batch attempt replaces collection_config before entering this function.
    """
    task = canonical_task(task)
    if task not in TASKS:
        return None
    previous = cfg.get("instruction_randomization")
    if previous is not None and previous.get("task") == task:
        record = dict(previous)
        if (record.get("schema") != SELECTION_SCHEMA
                or record.get("stream") != STREAM
                or type(record.get("template_index")) is not int
                or not 0 <= record["template_index"] < 20):
            raise ValueError("Invalid saved instruction selection")
        template = record.get("template")
        validate_template(task, template)
        if hashlib.sha256(template.encode()).hexdigest() != record.get("template_sha256"):
            raise ValueError("Saved instruction template checksum does not match")
        if record.get("template_id") != f"{task}/{record['template_index'] + 1:02d}":
            raise ValueError("Saved instruction template identity does not match")
    else:
        bank = load_bank()
        index = int(rng.integers(20))
        template = bank["tasks"][task][index]
        record = dict(schema=SELECTION_SCHEMA, bank_schema=bank["schema"],
                      task=task, seed=int(seed), stream=STREAM,
                      template_index=index, template_id=f"{task}/{index + 1:02d}",
                      template=template, template_sha256=hashlib.sha256(template.encode()).hexdigest())
    record["instruction"] = template.format(**(values or {}))
    cfg["instruction_randomization"] = record
    return record
