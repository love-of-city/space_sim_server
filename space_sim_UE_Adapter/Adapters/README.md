# Python 状态发送端

`bsk_render_adapter` 负责 Basilisk/MJScene 对象注册、坐标与姿态转换、manifest、最新帧网络发送和 `.bskrec` 录制，不包含 UE 代码。

安装与环境选择统一见[仓库 README](../README.md#1-环境与安装)，包含 uv、Conda、传统 venv/pip 三套步骤。从仓库根目录运行：uv 使用 `uv pip install --python $env:SPACE_SIM_PYTHON -e ".[test]"`；Conda/传统 venv 使用 `& $env:SPACE_SIM_PYTHON -m pip install -e ".[test]"`。两者均明确使用已选择的解释器，不需要更换环境管理方式。

安装后导入 `bsk_render_adapter.BasiliskRenderBridge`。任意场景可传入 `recording_path`；离线录制可配合 `RecordingOnlyPublisher`，无需启动渲染器。协议见[字段说明](../Unreal/BskUnrealRenderer/docs/PROTOCOL.md)。
