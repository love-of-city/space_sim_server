"""Satellite-driving evaluations within the existing authenticated platform."""
import csv
import io
import json
from typing import Literal

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse, Response

from .benchmark_jobs import BenchmarkCreate, BenchmarkManager


def numeric_metrics(value, prefix=""):
    """Keep nested orbital metrics and vector components in research exports."""
    if isinstance(value, dict):
        return {name: number for key, item in value.items()
                for name, number in numeric_metrics(item, f"{prefix}.{key}" if prefix else key).items()}
    if isinstance(value, (tuple, list)):
        return {name: number for index, item in enumerate(value)
                for name, number in numeric_metrics(item, f"{prefix}[{index}]").items()}
    return {prefix: value} if prefix and (type(value) in (float, int) or value is None) else {}


def results_csv(document):
    stream = io.StringIO(newline="")
    rows = document["job"]["summary"]["trials"]
    values = [numeric_metrics(row.get("physics_score") or {}) for row in rows]
    metrics = sorted({key for score in values for key in score})
    writer = csv.writer(stream)
    writer.writerow(["configuration", "method", "split", "selection", "trial_id", "seed", "outcome", "wall_time_s", *metrics])
    job = document["job"]
    for row, score in zip(rows, values, strict=True):
        writer.writerow([job["configuration"], job["method"], job["request"]["split"], job["request"]["selection"],
                         row["trial_id"], row["seed"], row["outcome"], row.get("wall_time_s"),
                         *(score.get(key) for key in metrics)])
    return stream.getvalue()


def install_benchmark_routes(app, config, current_user):
    manager = BenchmarkManager(config.project_root, config.data_root)
    app.state.benchmarks = manager
    router = APIRouter(prefix="/api/benchmarks")

    def call(action, *args):
        try:
            return action(*args)
        except KeyError:
            raise HTTPException(404, "找不到该评测记录") from None
        except (ValueError, OSError) as error:
            raise HTTPException(409, str(error)) from error

    @router.get("/catalog")
    def catalog(request: Request):
        current_user(request)
        return call(manager.catalog)

    @router.get("")
    def listing(request: Request):
        return {"jobs": call(manager.list, current_user(request))}

    @router.post("", status_code=201)
    def start(payload: BenchmarkCreate, request: Request):
        return call(manager.start, payload, current_user(request))

    @router.get("/{key}")
    def get(key: str, request: Request):
        return call(manager.get, key, current_user(request))

    @router.post("/{key}/stop")
    def stop(key: str, request: Request):
        return call(manager.stop, key, current_user(request))

    @router.post("/{key}/resume")
    def resume(key: str, request: Request):
        return call(manager.resume, key, current_user(request))

    @router.post("/{key}/rescore")
    def rescore(key: str, request: Request):
        return call(manager.rescore, key, current_user(request))

    @router.get("/{key}/export")
    def export(key: str, request: Request, format: Literal["json", "csv"] = "json"):
        document = call(manager.export, key, current_user(request))
        body = results_csv(document) if format == "csv" else json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False)
        return Response(body, media_type="text/csv" if format == "csv" else "application/json",
                        headers={"Content-Disposition": f'attachment; filename="{document["job"]["id"]}.{format}"'})

    @router.get("/{key}/trials/{trial}/artifacts/{name}")
    def artifact(key: str, trial: str, name: str, request: Request):
        path = call(manager.artifact, key, trial, name, current_user(request))
        return FileResponse(path, filename=f"{trial}-{path.name}", media_type="application/octet-stream")

    app.include_router(router)
    return manager
