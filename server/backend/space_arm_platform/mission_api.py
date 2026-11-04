"""Authenticated flight and mission routes; execution stays in the runtimes."""
from __future__ import annotations
import asyncio
from typing import Any, Literal
from fastapi import HTTPException, Request
from fastapi.responses import FileResponse
from pydantic import BaseModel, ConfigDict, Field, StrictBool
from .auth_api import current_user
from .models import SceneInstanceCreate
from .rendezvous_config import FlightOptions
from .mission_runtime import MISSION_MODES

class RendezvousStart(BaseModel):
    model_config = ConfigDict(extra="forbid")
    live: StrictBool = False
    mode: Literal['simulation'] = 'simulation'
    scenario: Literal['team-in-orbit', 'team-orbit-raise', 'near-field', 'near-field-resume'] = 'team-in-orbit'
    options: dict[str, Any] = Field(default_factory=dict)


class MissionStart(BaseModel):
    model_config = ConfigDict(extra="forbid")
    mode: Literal["teleop", "team-in-orbit", "team-orbit-raise", "near-field", "near-field-resume"] = "teleop"
    scene: SceneInstanceCreate = Field(default_factory=SceneInstanceCreate)
    options: dict[str, Any] = Field(default_factory=dict)
    live: StrictBool = True
    continuous: StrictBool = False


def install_mission_routes(app, *, missions, rendezvous, rendezvous_runtime, policy, stop_scene_executor):
    @app.get("/api/rendezvous/state")
    async def rendezvous_state() -> dict[str, Any]:
        return {**rendezvous.status(), "runtime": rendezvous_runtime.status()}


    @app.get("/api/rendezvous/figure/{name}")
    async def rendezvous_figure(name: str) -> FileResponse:
        allowed = {
            "rendezvous_summary.png",
            "rendezvous_contact.png",
            "full_mission_diagnostics.png",
        }
        if name not in allowed or rendezvous_runtime.output_dir is None:
            raise HTTPException(status_code=404, detail="figure unavailable")
        path = rendezvous_runtime.output_dir / name
        if not path.is_file():
            raise HTTPException(status_code=404, detail="figure unavailable")
        return FileResponse(path, media_type="image/png")


    @app.get("/api/rendezvous/artifact/{name}")
    async def rendezvous_artifact(name: str, mission_id: str | None = None, phase: str | None = None) -> FileResponse:
        if name not in {"mission_report.json", "handoff.json", "input_handoff.json", "model_manifest.json", "mission_options.json", "team_telemetry.npz", "gnss_measurements.npz"}:
            raise HTTPException(status_code=404, detail="Unknown rendezvous artifact")
        if mission_id:
            try:
                path = missions.artifact(mission_id, name, phase=phase)
            except (ValueError, OSError) as error:
                raise HTTPException(status_code=404, detail="任务结果不可用") from error
        else:
            if phase is not None:
                raise HTTPException(status_code=404, detail="阶段下载需要任务编号")
            if rendezvous_runtime.output_dir is None:
                raise HTTPException(status_code=404, detail="No rendezvous output")
            path = rendezvous_runtime.output_dir / name
        if not path.is_file():
            raise HTTPException(status_code=404, detail="Artifact is not ready")
        return FileResponse(path, filename=name)


    @app.post("/api/rendezvous/start")
    async def rendezvous_start(request: Request, payload: RendezvousStart | None = None) -> dict[str, Any]:
        policy.require_free()
        user = current_user(request)
        payload = payload or RendezvousStart()
        try:
            return await missions.start_flight(user, scenario=payload.scenario, options=payload.options,
                                                live=payload.live, mode=payload.mode)
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        except ValueError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error


    @app.get("/api/rendezvous/options")
    async def rendezvous_options() -> dict[str, Any]:
        return FlightOptions().to_dict()


    @app.post("/api/rendezvous/handoff")
    async def rendezvous_handoff(request: Request) -> dict[str, Any]:
        policy.require_free()
        try:
            return await missions.handoff(current_user(request))
        except PermissionError as error:
            raise HTTPException(status_code=403, detail=str(error)) from error
        except (ValueError, OSError, RuntimeError) as error:
            raise HTTPException(status_code=409, detail=str(error)) from error


    @app.post("/api/rendezvous/stop")
    async def rendezvous_stop(request: Request) -> dict[str, Any]:
        return await stop_mission(request)


    @app.get("/api/mission/catalog")
    async def mission_catalog():
        from .gnss_config import GNSSConfig
        receiver = GNSSConfig()
        defaults = FlightOptions().to_dict()
        from .orbital_physics import OrbitalPhysics
        defaults["orbital_physics"] = OrbitalPhysics().model_dump(mode="json")
        defaults.update(differential_gnss=receiver.to_dict(), gnss_position_sigma_m=receiver.absolute_position_sigma_m,
                        gnss_velocity_sigma_m_s=receiver.absolute_velocity_sigma_m_s)
        return {"modes": MISSION_MODES, "flight_defaults": defaults, "continuous_default": True}


    @app.get("/api/mission/state")
    async def mission_state(request: Request):
        return await asyncio.to_thread(missions.status, current_user(request))


    @app.post("/api/mission/start", status_code=202)
    async def start_mission(payload: MissionStart, request: Request):
        policy.require_free()
        user = current_user(request)
        try:
            if payload.mode == "teleop":
                return await missions.start_scene(payload.scene, user, background=True)
            from .scene_targets import SELF_COLLISION_TEMPLATE
            if payload.scene.template_id != SELF_COLLISION_TEMPLATE:
                raise ValueError("交会任务仅支持团队 SARM + 地面验证星默认模型")
            return await missions.start_flight(user, scenario=payload.mode, options=payload.options,
                                                live=payload.live, background=True, continuous=payload.continuous)
        except ValueError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error


    @app.post("/api/mission/handoff", status_code=202)
    async def handoff_mission(request: Request):
        policy.require_free()
        try:
            return await missions.handoff(current_user(request), background=True)
        except PermissionError as error:
            raise HTTPException(status_code=403, detail=str(error)) from error
        except (ValueError, OSError, RuntimeError) as error:
            raise HTTPException(status_code=409, detail=str(error)) from error


    @app.post("/api/mission/stop")
    async def stop_mission(request: Request):
        try:
            return await missions.stop(current_user(request), lambda: stop_scene_executor(request))
        except PermissionError as error:
            raise HTTPException(status_code=403, detail=str(error)) from error
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
