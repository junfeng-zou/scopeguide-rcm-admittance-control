# ScopeGuide：基于六维力传感器的 RCM 三自由度导纳拖拽

ScopeGuide 是一套面向机器人辅助内窥镜操作的 MATLAB 实时控制程序。系统读取安装在
机械臂末端的 OnRobot HEX-H 六维力/力矩传感器，通过重力与零偏补偿、滤波、死区、
导纳模型和 RCM 约束 QP，将操作者施加的力转换为机械臂关节位置指令，实现以下三个
自由度的手动拖拽：

- `pivot1`：绕 RCM 点的第一横向偏转自由度；
- `pivot2`：绕 RCM 点的第二横向偏转自由度；
- `insertion`：沿内窥镜镜杆轴线的插入/退出自由度。

Roll 自由度保持关闭。当前仓库是已经整理过的最终运行版本，仅包含主程序、最终参数
以及运行所需依赖，不包含开发阶段的测试、验收、离线复核和通信诊断程序。

> **安全声明**：本项目是科研原型，不是医疗器械，不得直接用于临床、人体或未经验证
> 的组织操作。键盘使能是锁存式软件输入，不是物理 deadman。运行时必须保持示教器和
> 急停可触及，并由操作者独立确认机械臂状态、工具安装、RCM、工作空间和安全阈值。

## 系统结构

```text
HEX-H（约 200 Hz）
        │
        ▼
质量/质心重力补偿 + 六维零偏补偿
        │
        ▼
启动软件置零 + 空闲时受限基线跟踪
        │
        ▼
20 Hz Fast 滤波（安全）+ 5 Hz Slow 滤波（控制）
        │
        ▼
力/力矩连续死区
        │
        ▼
RCM 广义作用量映射 [pivot1, pivot2, insertion]
        │
        ▼
三自由度导纳模型
        │
        ▼
软 RCM 约束 QP + 关节限速/限加速度
        │
        ▼
速度积分为关节目标 → ServoJ（20 Hz）
```

控制计算运行在独立的 MATLAB process worker 中，图形界面在客户端进程中刷新，避免
实时绘图阻塞控制周期。

## 硬件配置

当前配置对应以下系统：

- 机械臂：Dobot CR5；
- 机械臂地址：`192.168.50.105`；
- Dashboard / Servo / Feedback 端口：`29999 / 30003 / 30004`；
- 六维力传感器：OnRobot HEX-H QC；
- 传感器地址：`192.168.50.201:502`；
- 传感器采样率：约 `200 Hz`；
- 内窥镜镜杆方向：法兰坐标系 `+Z`；
- RCM 坐标（机械臂基座系）：`[-0.2910, 0.6316, 0.1378] m`。

归档内使用的法兰到内窥镜末端变换为：

```matlab
TFlangeEndoscope = [ ...
    1  0  0  -0.002840;
    0  1  0   0.113051;
    0  0  1   0.355046;
    0  0  0   1.000000];
```

这些几何参数仅对当前机械安装有效。更换夹具、内窥镜、连接件、传感器安装方向或 RCM
位置后，必须重新核对几何、质量、质心、六维零偏和 RCM 坐标。

## 软件依赖

本版本已在 MATLAB R2025b 环境完成独立依赖检查，需要：

- MATLAB；
- Optimization Toolbox（提供 `quadprog`）；
- Parallel Computing Toolbox（提供 process pool）；
- 支持 `tcpclient` 的 MATLAB 环境；
- Linux 主机与机械臂、HEX-H 位于可互通的局域网。

MATLAB 静态分析得到的全部项目依赖均已包含在本目录中，配置、标定和运行结果路径也都
相对于仓库根目录解析，不依赖原始开发目录。

## 目录结构

```text
.
├── run_scopeguide_3dof_drag.m       # 三自由度拖拽主程序
├── scopeguide_3dof_drag_config.m    # 最终参数入口
├── +scopeguide/                     # 控制、RCM、QP、安全、I/O 与 UI
├── config/                          # 默认配置与配置校验
├── robot/                           # Dobot TCP/IP 与 ServoJ 通信
├── force_sensor/+onrobot/           # HEX-H Modbus TCP 采集与滤波
├── force_calibration/               # 重力和零偏补偿
├── resources/
│   ├── validated_base_config.mat    # 已验证的基础配置
│   └── force_calibration.json       # 当前负载标定
└── MANIFEST_SHA256.txt              # 文件完整性校验清单
```

## 当前控制参数

最终参数集中在 `scopeguide_3dof_drag_config.m` 的 `USER-EDITABLE FINAL VALUES`
区域。

| 参数 | 当前值 |
|---|---:|
| 控制频率 | 20 Hz |
| 全局 SpeedScale | 120% |
| ServoJ `t` | 0.05 s |
| ServoJ `lookahead_time` / `gain` | 60 / 400 |
| Dobot SpeedFactor | 20% |
| EnableRobot 负载 | 1.3 kg |
| Pivot 设计力 | 5 N |
| Pivot 目标/最大速度 | 2.133 / 3.0 deg/s |
| Pivot 时间常数 | 0.12 s |
| Pivot 相对运动范围 | 7.5 deg |
| Insertion 设计力 | 5 N |
| Insertion 目标/最大速度 | 6 / 8 mm/s |
| Insertion 时间常数 | 0.30 s |
| Insertion 相对运动范围 | ±15 mm |
| 关节速度/加速度上限 | 6 deg/s / 24 deg/s² |
| ServoJ 单周期目标步长上限 | 0.30 deg |
| 目标—反馈误差阈值 | 0.50 deg |
| RCM 模式 | soft |
| RCM soft / hard 半径 | 0.5 / 2.0 mm |
| 力/力矩死区 | 1.3 N / 0.18 N·m |
| 原始/Fast 合力停止阈值 | 60 / 40 N |
| 原始/Fast 合力矩停止阈值 | 16 / 8 N·m |

`SpeedScale` 在当前最终配置中主要承担配置记录作用。由于参数已经以最终值写入，不要只
修改 `SpeedScale` 来期待实际速度同比变化；应直接修改对应的 Pivot/Insertion 目标速度、
速度上限、加速度或导纳时间常数，并重新完成安全验证。

## 快速开始

### 1. 获取代码

```bash
git clone https://github.com/junfeng-zou/scopeguide-rcm-admittance-control.git
cd scopeguide-rcm-admittance-control
```

可选：检查归档文件是否完整。

```bash
sha256sum -c MANIFEST_SHA256.txt
```

### 2. 运行前检查

1. 确认末端机械结构与标定时保持一致；
2. 确认机械臂处于 TCP/IP 控制模式；
3. 确认主机可以访问 `192.168.50.105` 和 `192.168.50.201`；
4. 清空内窥镜外部载荷并保持线缆松弛；
5. 确认运动空间无障碍，示教器和急停可立即触及；
6. 启动软件置零期间保持机械臂和末端静止，不要按 Space。

### 3. MATLAB 启动命令

```matlab
pool = gcp('nocreate');
if ~isempty(pool)
    delete(pool);
end

clear functions;
cd('/path/to/scopeguide_3dof_admittance_drag_final_20260821');

[cfgDrag, profile] = scopeguide_3dof_drag_config();

[dragSummary, dragResultDir] = run_scopeguide_3dof_drag( ...
    Config=cfgDrag, ...
    DurationSec=120, ...
    WindowSec=20, ...
    PlotRateHz=10, ...
    DofMode="full_3dof", ...
    ArmPhysicalMotion=true, ...
    MotionConfirmation="ENABLE_SCOPEGUIDE_MOTION", ...
    SetupConfirmation="SCOPEGUIDE_3DOF_DRAG_READY", ...
    SafetyThresholdConfirmation= ...
        "CONFIRM_LIMITS_60N_40N_16NM_8NM", ...
    SecondObserverPresent=true, ...
    FixtureAndClearanceConfirmed=true);
```

程序会依次执行：

1. 创建一个 process worker 并预热 `quadprog`；
2. 连接机械臂；
3. 发送 `EnableRobot(1.3)` 和 `SpeedFactor(20)`；
4. 通过 `30004` 反馈确认使能和速度比例；
5. 连接 HEX-H；
6. 完成启动软件置零；
7. 等待操作者使能拖拽；
8. 结束或退出时停止 ServoJ 并调用 `DisableRobot()`。

## 运行操作

| 输入 | 行为 |
|---|---|
| `Space` | 将软件运动使能锁存为 ON；区域变绿后允许力输入产生运动 |
| `Esc` | 立即将软件运动使能置为 OFF |
| `R` | 撤力并关闭使能后，对可恢复 Fault 执行复位 |
| 关闭窗口 | 请求停止程序并执行清理 |

软件锁存不能替代物理 deadman。发现运动异常时优先使用 `Esc`、示教器停止或急停，不要
依赖 MATLAB 图形界面作为唯一安全措施。

## 数据记录

每次运行会在仓库内自动创建：

```text
results/scopeguide_3dof_drag_full_3dof_<timestamp>/
```

主要输出包括：

- `drag_control.csv`：控制周期、力、QP、关节速度、RCM 误差和状态机记录；
- `drag_worker_result.mat`：完整 MATLAB 运行数据；
- `summary.json` / `summary_worker.json`：频率、ServoJ、跟踪误差和停止原因摘要；
- `config_snapshot.json`：本次实验实际使用的完整配置；
- `authorization.json`：实机运动授权门检查结果；
- `command_fault_diagnostics.json`：仅在运动命令通道异常时生成。

程序结束后曲线窗口默认保留，可手动查看和关闭。

## 常见问题

### 按 Space 后不能进入绿色状态

首先确认启动软件置零已经完成。置零要求机械臂静止、外力较小且反馈有效；若持续施力、
线缆受拉或反馈不稳定，实际等待时间可能超过名义的 3 秒。

### 进入 `FAST_FORCE_STOP` 或 `FAST_MOMENT_STOP`

表示 Fast 滤波后的合力或合力矩超过安全阈值。先撤力、按 `Esc`，确认原因消失后再按
`R`。不要在没有实验依据的情况下直接提高安全阈值。

### Insertion 有力但速度为零

检查是否已经达到 `±15 mm` 相对插入行程边界。如果位于正向边界，继续施加同方向轴向
力会被行程限制器钳制为零。

### 找不到 `quadprog` 或无法创建 process pool

确认 Optimization Toolbox 和 Parallel Computing Toolbox 已安装并获得许可。

### 无法连接机械臂或 HEX-H

检查 IP、网段、防火墙、机械臂 TCP/IP 模式和 HEX-H Modbus TCP 连接。不要在连接状态
不明确时反复发送运动命令。

## 已知限制

- 当前输入设备是键盘/鼠标软件使能，没有独立物理 deadman；
- RCM 点和工具变换依赖当前固定安装，结构改变后不会自动更新；
- 力传感器温漂通过启动置零和受限基线跟踪缓解，但不能替代正确安装和热稳定；
- RCM 使用软约束，`2 mm` 是当前 hard 半径监控值，不等同于机械硬限位；
- 当前版本面向自由空间科研调试，未验证组织接触和临床安全性。

## 完整性与发布

仓库附带 `MANIFEST_SHA256.txt`。修改任何源码或资源后应重新生成清单：

```bash
find . -path './.git' -prune -o \
  -type f ! -name MANIFEST_SHA256.txt -print0 \
  | sort -z \
  | xargs -0 sha256sum > MANIFEST_SHA256.txt
```

当前归档未附带开源许可证。正式发布到 GitHub 前，请根据代码使用范围补充合适的
`LICENSE`，并避免提交与实验无关的个人信息、网络凭据或敏感数据。
