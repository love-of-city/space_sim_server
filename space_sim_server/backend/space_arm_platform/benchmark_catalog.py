"""Versioned autonomy requirements, deliberately separate from runtime readiness.

The teacher's first and final lists reuse source IDs for different tasks. Only
the final capture-bounded source namespace below is accepted by this catalogue.
Listing a requirement never grants an execution adapter or a validation pass.
"""
from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, StrictStr, field_validator, model_validator


CATALOG_PATH = Path(__file__).resolve().parents[2] / "config" / "benchmarks" / "autonomy-catalog-v1.json"
SOURCE_ID = "teacher-capture-scope-122-v1"
SOURCE_COUNTS = {"S": 26, "D": 62, "M": 24, "T": 10}
Priority = Literal["P0", "P1", "P2", "P3"]
VisionMode = Literal["V0", "V1", "V2", "V3"]
Cooperation = Literal["C", "P", "N", "none"]
Family = Literal[
    "attitude", "orbit", "navigation", "safety", "vision", "autonomy",
    "resources", "recovery", "relative-navigation", "rendezvous",
    "relative-motion", "formation", "approach", "inspection", "docking",
    "grasp", "coordination", "mission",
]


class CatalogModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class Source(CatalogModel):
    id: Literal["teacher-capture-scope-122-v1"]
    description: StrictStr = Field(min_length=1)
    expected_counts: dict[str, int]

    @field_validator("expected_counts", mode="before")
    @classmethod
    def source_counts_are_fixed(cls, value):
        if (not isinstance(value, dict) or value != SOURCE_COUNTS
                or any(type(count) is not int for count in value.values())):
            raise ValueError("the final source contains S26, D62, M24 and T10")
        return value


class Scope(CatalogModel):
    endpoint: Literal["verified_docking_or_grasp_connection"]
    excluded: tuple[Literal[
        "post_capture_combined_body_control", "towing", "repair", "refueling",
        "assembly", "release",
    ], ...]

    @field_validator("excluded")
    @classmethod
    def complete_exclusions(cls, value):
        expected = {"post_capture_combined_body_control", "towing", "repair",
                    "refueling", "assembly", "release"}
        if len(value) != len(expected) or set(value) != expected:
            raise ValueError("capture-bounded scope must retain every exclusion")
        return value


class TaskRequirement(CatalogModel):
    id: StrictStr = Field(pattern=r"^ssb\.[a-z][a-z0-9-]*\.[a-z][a-z0-9-]*$")
    source_id: StrictStr = Field(pattern=r"^[SDMT][0-9]{2}$")
    title: StrictStr = Field(min_length=1)
    priority: Priority
    family: Family
    requested_vision_modes: tuple[VisionMode, ...] = Field(min_length=1)
    target_cooperation: tuple[Cooperation, ...] = Field(min_length=1)
    classification_origin: Literal["source", "source_with_explicit_design_tags"]

    @field_validator("title")
    @classmethod
    def title_has_content(cls, value):
        if not value.strip() or value != value.strip():
            raise ValueError("task title must be nonblank and already trimmed")
        return value

    @field_validator("requested_vision_modes", "target_cooperation")
    @classmethod
    def unique_labels(cls, value):
        if len(set(value)) != len(value):
            raise ValueError("classification labels cannot be repeated")
        return value

    @model_validator(mode="after")
    def consistent_classification(self):
        if "none" in self.target_cooperation and self.target_cooperation != ("none",):
            raise ValueError("no specified target cannot also be a cooperative target")
        prefix, number = self.source_id[0], int(self.source_id[1:])
        if not 1 <= number <= SOURCE_COUNTS[prefix]:
            raise ValueError("source ID lies outside the final capture-bounded list")
        if (prefix == "T") != (self.family == "mission"):
            raise ValueError("T entries are the end-to-end mission requirements")
        expected_origin = "source_with_explicit_design_tags" if prefix in "MT" else "source"
        if self.classification_origin != expected_origin:
            raise ValueError("derived multi-spacecraft/mission tags must disclose their origin")
        if prefix == "S" and self.target_cooperation != ("none",):
            raise ValueError("single-spacecraft source tasks have no specified target satellite")
        if prefix in "DM" and "none" in self.target_cooperation:
            raise ValueError("dual/multi-spacecraft tasks must classify the target relationship")
        return self


class AutonomyCatalog(CatalogModel):
    schema_version: Literal["spacesim-autonomy-catalog/1"] = Field(alias="schema")
    source: Source
    scope: Scope
    tasks: tuple[TaskRequirement, ...]

    @model_validator(mode="after")
    def exact_source_coverage(self):
        source_ids = [task.source_id for task in self.tasks]
        internal_ids = [task.id for task in self.tasks]
        expected = {f"{prefix}{index:02}" for prefix, count in SOURCE_COUNTS.items()
                    for index in range(1, count + 1)}
        if len(internal_ids) != len(set(internal_ids)):
            raise ValueError("stable task IDs must be unique")
        if len(source_ids) != len(set(source_ids)) or set(source_ids) != expected:
            missing = sorted(expected - set(source_ids))
            raise ValueError(f"the catalogue must cover each final source task exactly once; missing={missing}")
        return self

    def task(self, task_id: str) -> TaskRequirement:
        """Resolve a stable ID or a source ID within this explicitly loaded source."""
        for requirement in self.tasks:
            if task_id in (requirement.id, requirement.source_id):
                return requirement
        raise KeyError(f"unknown task in {self.source.id}: {task_id}")

    def select(self, *, priority: Priority | None = None,
               family: Family | None = None, vision: VisionMode | None = None,
               source_group: Literal["S", "D", "M", "T"] | None = None):
        return tuple(task for task in self.tasks
                     if (priority is None or task.priority == priority)
                     and (family is None or task.family == family)
                     and (vision is None or vision in task.requested_vision_modes)
                     and (source_group is None or task.source_id.startswith(source_group)))

    def coverage(self) -> dict:
        """Requirement counts only: no inference about implemented environments."""
        return {
            "source_id": self.source.id,
            "requirements": len(self.tasks),
            "by_source_group": dict(sorted(Counter(t.source_id[0] for t in self.tasks).items())),
            "by_priority": dict(sorted(Counter(t.priority for t in self.tasks).items())),
            "by_family": dict(sorted(Counter(t.family for t in self.tasks).items())),
            "scope": "requirement_catalogue_only",
        }


def load_catalog(path: Path | str = CATALOG_PATH) -> AutonomyCatalog:
    return AutonomyCatalog.model_validate_json(Path(path).read_text(encoding="utf-8"))
