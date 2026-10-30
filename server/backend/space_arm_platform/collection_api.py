"""Authenticated collection control and dataset playback endpoints."""

from pathlib import Path

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse, Response

from .collection_jobs import CollectionCreate, CollectionManager, TASKS
from .control_defaults import DEFAULT_OPERATING_JOINT_DEG, ZERO_START_TELEOP_PROFILE
from .dataset_view import Store
from .models import SceneInstanceCreate
from .collection_randomization import randomization_catalog
from .collection_layouts import LayoutPool
from .collection_inputs import requires_color_layout
from .overview_camera import overview_camera_catalog
from .scene_runtime import SceneRuntimeManager
from .sampling import DYNAMICS_HZ, DEFAULT_IK_HZ, DEFAULT_CAPTURE_HZ, WRENCH_HZ


def install_collection_routes(app, config, scenes, current_user):
    manager = CollectionManager(
        config.project_root,
        config.data_root,
        config.runtime_adapter_root,
        config.runtime_unreal_root,
    )
    store = Store(config.data_root)
    collection_scenes = SceneRuntimeManager(None, project_root=config.project_root)
    app.state.collections = manager
    router = APIRouter(prefix="/api")

    def episode_for(key, user):
        try:
            path = store.episode(key)
            meta_path = (path / "metadata.json").resolve()
            if not meta_path.is_relative_to(path):
                raise FileNotFoundError(key)
            meta = store.metadata(path)
        except (ValueError, OSError):
            raise HTTPException(404, "找不到该记录") from None
        if user["role"] != "admin" and meta.get("operator_user_id") != user["user_id"]:
            raise HTTPException(404, "找不到该记录")
        return path

    @router.get("/collections/catalog")
    def catalog():
        layout = dict(available=False, count=0, description="尚未配置随机布局池")
        if manager.layout_pool is not None:
            try:
                pool = LayoutPool(manager.layout_pool)
                layout.update(available=True, count=len(pool.document["layouts"]),
                              description="逐条随机选择布局；13 个安装单元独立移动，可含台面内朝向变化；机构初始状态保持。")
            except (ValueError, OSError, KeyError) as error:
                layout["description"] = f"布局池未通过校验：{error}"
        return dict(
            tasks=[
                dict(id=key, label=task.label, description=task.description,
                     requires_color_layout=requires_color_layout(key))
                for key, task in TASKS.items()
            ],
            max_workers=4,
            rates=dict(
                dynamics=DYNAMICS_HZ,
                ik=DEFAULT_IK_HZ,
                main=DEFAULT_CAPTURE_HZ,
                wrench=WRENCH_HZ,
            ),
            cameras=dict(count=2, width=1024, height=1024),
            randomization_options=randomization_catalog(),
            layout_randomization=layout,
            fixed_layout=manager.fixed_layout_capability(),
            overview_camera_randomization=overview_camera_catalog(),
            wrist_wrench_noise=False,
            disturbances_enabled=False,
            lighting_randomized=False,
            baseline_joint_deg=list(DEFAULT_OPERATING_JOINT_DEG),
        )

    @router.get("/collections")
    def list_collections(request: Request):
        return {"jobs": manager.list(current_user(request))}

    @router.post("/collections", status_code=201)
    def create_collection(payload: CollectionCreate, request: Request):
        user = current_user(request)
        try:
            # A new immutable scene config; creating it does not launch or stop the interactive scene.
            scene = collection_scenes.create_instance(
                SceneInstanceCreate(
                    template_id=TASKS[payload.task].template,
                    randomization_profile=ZERO_START_TELEOP_PROFILE,
                    operating_arm_joint_position_deg=list(DEFAULT_OPERATING_JOINT_DEG),
                    seed=payload.seed,
                ),
                created_by=user,
            )
            return manager.start(payload, user, scene)
        except ValueError as error:
            raise HTTPException(409, str(error)) from error

    @router.get("/collections/{key}")
    def get_collection(key: str, request: Request):
        try:
            return manager.get(key, current_user(request))
        except KeyError:
            raise HTTPException(404, "找不到该采集任务") from None

    @router.post("/collections/{key}/stop")
    def stop_collection(key: str, request: Request):
        try:
            return manager.stop(key, current_user(request))
        except KeyError:
            raise HTTPException(404, "找不到该采集任务") from None

    @router.get("/datasets")
    def datasets(request: Request):
        user = current_user(request)
        listing = store.listing()
        visible = []
        for record in listing["episodes"]:
            try:
                episode_for(record["id"], user)
            except HTTPException:
                continue
            visible.append(record)
        return {**listing, "episodes": visible}

    @router.get("/datasets/{key}")
    def dataset(key: str, request: Request):
        path = episode_for(key, current_user(request))
        for relative in ("steps.jsonl", "lerobot/high_frequency/wrist_wrench.jsonl"):
            if not (path / relative).resolve().is_relative_to(path):
                raise HTTPException(404, "无效的数据路径")
        try:
            return Response(store.load(key), media_type="application/json")
        except (OSError, ValueError, KeyError) as error:
            raise HTTPException(422, f"无法读取该记录：{error}") from error

    @router.get("/datasets/{key}/media/{relative:path}")
    def media(key: str, relative: str, request: Request):
        path = episode_for(key, current_user(request))
        file = (path / relative).resolve()
        if (
            not file.is_relative_to(path / "lerobot" / "videos")
            or file.suffix.lower() != ".mp4"
            or not file.is_file()
        ):
            raise HTTPException(404, "找不到视频")
        return FileResponse(file, media_type="video/mp4")

    app.include_router(router)

    return manager
