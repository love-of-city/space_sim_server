# 系统边界

server/backend/space_arm_platform 提供API、用户、场景和采集进程管理。collection 负责采集编排、会话、录制和恢复。simulation 负责权威动力学、任务控制和规划；native 提供原生运算。UE负责渲染与相机产品。

日常入口 server/tools/collect.py 调用 collection.sequence，经 collect_scripted_tasks 管理会话，再进入单任务录制/执行和任务控制器。网页/API批采与该CLI分别验收。

模型、配置、任务版本和实际加载库指纹共同定义运行。worker复用保留每条状态复位和异常回收边界；编译模型缓存不能复用上一条物理状态。

协议定义在 server/contracts，发送端在 ue-adapter/Adapters，UE实现位于 Unreal/BskUnrealRenderer/Plugins/BskUnrealRuntime。
