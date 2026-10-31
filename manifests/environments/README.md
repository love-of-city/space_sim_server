# 已观察到的环境

2026-10-30从现有仿真和几何环境导出。Python 3.13.16；真实Basilisk/MuJoCo ABI与UE构建另行核验。去除本机editable路径和file URL；从仓库根安装server和ue-adapter。此列表是来源快照，未声称已在干净机器重装验证，也不包含UE/驱动/系统库。

## 运行策略与环境重建

`linux-runtime.json` 记录本机实际引擎 Build 和关键插件指纹，并声明当前 Python ABI 要求；`windows-runtime.json` 是待验收 Windows 的版本策略，不是Windows实测证书。两者分开记录 clean-install 与 GPU 状态。

`tools/release/resolve_environment.py` 可生成带 SHA 的解析锁，并在新的隔离目录安装和检查。只有成功证据才能升级为经过重装认证的安装锁；本次网络/缓存条件未完成重装，因此没有发布名不副实的 `.lock`。详见 [发布重建](../../docs/guides/release-reproduction.md)。
