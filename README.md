# ScopeGuide：RCM 与 Cartesian 导纳拖拽

面向 Dobot CR5 与 OnRobot HEX-H 的 MATLAB 控制程序，包含两种拖拽模式：

- **RCM 三自由度**：绕固定 RCM 点的双轴 pivot 与沿镜杆的 insertion。
- **Cartesian 六自由度**：无 RCM 约束的平移与旋转拖拽。

控制流程为力/力矩采集与补偿、滤波和死区、导纳模型、速度 QP、关节目标积分与 ServoJ。两种模式共享硬件接口和保护模块，使用各自的几何约束与导纳参数。

## 版本与入口

| 位置 | 内容 | 使用说明 |
|---|---|---|
| [`current/`](current/) | 当前 RCM 与 6DOF 工作版本、配置、必要资源及测试 | [启动命令](current/docs/admittance_drag_launch_commands.md) |
| [`archives/scopeguide_3dof_admittance_drag_final_20260821/`](archives/scopeguide_3dof_admittance_drag_final_20260821/) | 原工作目录 `archives` 中的三自由度归档快照，包含归档中的后续本地修改 | [归档说明](archives/scopeguide_3dof_admittance_drag_final_20260821/README.md) |
| 根目录原有 `.m`、`+scopeguide/` 等文件 | 2026-08-21 已发布的初始三自由度版本，保留原入口 | [初始版本说明](docs/initial_release_readme.md) |

最新开发请使用 `current/`。各版本是独立的 MATLAB 工程，不能同时加入路径；尤其不要对整个仓库执行 `addpath(genpath(...))`。建议切换版本时重启 MATLAB。

## 开始使用

```bash
git clone git@github.com:junfeng-zou/scopeguide-rcm-admittance-control.git
cd scopeguide-rcm-admittance-control/current
```

在 MATLAB 中将当前目录设为 `current/`，按[启动说明](current/docs/admittance_drag_launch_commands.md)选择模式。软件依赖及离线测试方法见 [`current/README.md`](current/README.md)。

配置文件附带原实验的工具几何、RCM 坐标、标定及基础配置。更换安装或实验装置后需要重新核对这些参数。实机入口会在运动确认门通过后使能机器人，软件键盘使能不能替代硬件急停。

## 发布检查

本次整理保留原控制参数和控制算法，调整了默认资源路径，并补充文档、归档副本和测试所需资源。完整实验结果、视频和临时文件未纳入仓库。检查范围与结果见[发布验证记录](current/docs/publication_validation.md)。

文件校验：

```bash
sha256sum -c MANIFEST_SHA256.txt
```

本仓库未新增开源许可授权；已有来源说明和版权信息保留在对应文件中。
