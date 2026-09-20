# 阶段 7：传感器 + 机器人实机只读 dry-run

## 当前状态

阶段 7 已进入“进行中”，只读主循环和离线安全测试已完成，尚未做 10 分钟
实机记录和人工方向确认，因此不标记为“已通过”。

用户提供的 RCM 点已按“机器人基座坐标系、单位 m”记录：

```matlab
pRcmBase = [-0.2906; 0.6310; 0.1388];  % m
```

版本化记录位于 `config/manual_rcm_point_stage07_20260813.json`。该点是人工空间标记，
不是多轴线拟合标定，所以继续保持：

```text
CalibrationValid = true   % 2026-08-14 用户确认，限当前不变装配
BoundsValidated  = true   % 0.5/2.0 mm 由用户接受用于自由空间 commissioning
EnableMotion     = false
DryRun           = true
```

工具几何、RCM 点/边界和力参数现按用户实物经验确认为通过，并记录在
`config/stage08_user_attestation_20260814.json`。该确认仅适用于当前不变装配和自由空间
commissioning；`ServoTimingVerified`、`SafetyThresholdsValidated` 和
`JointLimitsValidated` 仍持续阻断阶段 8 的真实运动，不影响阶段 7 只读预测。

## 已实现链路

`run_stage07_live_readonly_dryrun.m` 同时连接 HEX-H 和机器人反馈，运行：

```text
原始六维力
  -> 零偏/重力补偿
  -> 20 Hz fast + 5 Hz slow
  -> AutomaticBaseline + 死区
  -> 工具几何和当前 RCM 误差
  -> 阶段 4 FSM
  -> 六维 wrench 从 HEX-H 原点平移到 RCM
  -> 3-DOF pivot/pivot/insertion 导纳（roll 关闭）
  -> 阶段 6 quadprog
  -> qdotPredicted / qTargetPredicted
  -> 同步 CSV 日志和低频绘图
```

为避免 MATLAB 图形刷新阻塞控制周期，新增的推荐入口是
`run_stage07_live_readonly_parallel.m`。它把链路拆分为：

```text
MATLAB client：键盘/鼠标 + 5 Hz 三幅图
        ⇅ PollableDataQueue（输入心跳 / 最新显示快照）
后台 worker：HEX-H + 机器人只读反馈 + 基线 + FSM + 导纳 + QP + CSV
```

worker 不创建 figure；client 不持有 HEX-H、机器人或控制器对象。worker 只按绘图频率
发送紧凑数值快照。client 每次取尽消息队列，只绘制最新快照，旧显示帧可以丢弃，不能
反向拖慢采集和控制。软件输入心跳默认是 50 Hz/250 ms 超时；界面退出、崩溃或长时间
无响应后 worker 会撤销软件使能。该软件输入仍然明确设置
`SupportsPhysicalMotion=false`，不能作为阶段 8 的实体安全按钮。

当前机器的 MATLAB R2025b 安装包含 `backgroundPool`、消息队列和 `parfeval`，但没有
安装完整的 Parallel Computing Toolbox，因此 `parpool("Processes",1)` 不可用。
`ExecutionBackend="auto"` 会在当前机器选择独立后台线程；以后安装完整 PCT 后会优先
选择独立 MATLAB 进程。也可显式指定 `"thread"` 或 `"process"`。

运行器只调用 `RobotAdapter.connectReadOnly/readState`，不存在 `ServoJ` 或
`sendServoTarget` 调用点。运行摘要同时记录机器人命令尝试数和发送数，两者都必须为 0。

## 运行顺序

### 推荐：同时绘图的解耦时序预检

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');

[parallelPreview, parallelDir] = ...
    run_stage07_live_readonly_parallel( ...
        DurationSec=60, ...
        PlotRateHz=5, ...
        StatusRateHz=2, ...
        ExecutionBackend="auto", ...
        FaultInjectionProfile="none");
```

这次运行仍显示原来的三幅实时图，但绘图不在控制 worker 中执行。结果目录名是
`stage07_live_readonly_parallel_<timestamp>`。`summary.json` 除原有采集、控制和 QP
时序外，还记录：

```text
ParallelExecutionBackend
ControlAndGraphicsDecoupled
WorkerDisplaySnapshotCount / WorkerDisplaySendDuration
ClientTelemetryReceivedCount / RenderedCount / DroppedCount
ClientRenderDuration
UiHeartbeatTimeoutCount / MaximumUiHeartbeatAgeSec
```

验收时重点比较并行版的 `ControlDt.P95/P99/Maximum` 和
`ControlDeadlineMissCount`。绘图耗时只应反映在 `ClientRenderDuration`，不应再进入
worker 的 `ControlDt`。如果并行版仍发生 `CONTROL_PERIOD_OVERRUN`，则问题不再是
`animatedline/drawnow`，需继续检查 Modbus、机器人反馈回调或系统调度。

2026-08-14 根据实机 TCP/IP 反馈约 33 Hz 的观测，默认主控制频率从 100 Hz 调整为
30 Hz（标称周期 33.33 ms）。机器人反馈 stale 门槛为 80 ms，控制周期硬上限为
75 ms，QP 单次求解预算为 15 ms。力传感器采样仍为 200 Hz，客户端绘图仍建议
5--10 Hz。历史 100 Hz 结果只作为旧配置的时序证据，调整后需要重新完成阶段 7
的短时预检和故障注入。

为避免 Linux/X11 的 `KeyRelease` 丢失、伪释放和自动重复问题，阶段 7 键盘采用幂等锁存
命令：按一次 `SPACE` 设置软件预测使能 ON，`Esc` 设置 OFF，完全忽略 `KeyRelease`。
重复 `KeyPress` 只会重复设置 ON，不会反复切换。鼠标蓝色区域仍采用按住使能、松开停止。
键盘锁存只允许 dry-run；未来真实运动必须使用独立物理 deadman。

### 0. 关闭曲线的纯控制链时序测试

如果需要区分控制周期 overrun 来自实时曲线还是 Modbus 读取，先运行：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');

[timing, timingDir] = run_stage07_live_readonly_dryrun( ...
    DurationSec=30, ...
    EnablePlot=false, ...
    PlotRateHz=5, ...
    StatusRateHz=2, ...
    FaultInjectionProfile="none");
```

`EnablePlot=false` 时不创建三幅曲线，不调用 `animatedline`、`addpoints`或
`xlim`。会保留一个输入/状态小窗口，用于接收 `SPACE` 锁存 ON、`Esc` OFF、鼠标按住
和 `R` 复位；
这个窗口不绘制数据曲线。颜色含义为蓝色 `DISABLED`、黄色 `PREARM`、绿色
`ENABLED`、红色 `FAULT`、橙色 `STOPPING`。

`summary.json` 中会额外记录：

```text
PlotEnabled
CurvePlotUpdateCount
DashboardStatusUpdateCount
DashboardUpdateDuration
```

将这次的 `ControlDt.P95/P99/Maximum`与开启曲线时对比。如果关图后 overrun 消失，
主因是同步绘图；如果仍出现 30–70 ms 长周期，则继续检查 Modbus 读取。

### 1. 30 秒短时预检

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');

[preview, previewDir] = run_stage07_live_readonly_dryrun( ...
    DurationSec=30, ...
    FaultInjectionProfile="none");
```

这一同步入口保留作回退和 A/B 对照；日常同时绘图测试优先使用上面的并行入口。

开始后前 3 s 保持机器人和内窥镜静止、不施力，等待启动软件置零。置零完成后：

- `SPACE`：锁存预测计算 ON；松开空格不会改变状态；
- `Esc`：撤销键盘锁存和鼠标请求，预测输出归零并重锚目标；
- 鼠标按住蓝色区域：仅在按住期间开放预测计算，松开即撤销；
- `R`：在松开输入且故障消失后复位锁存故障；
- `Esc`：释放软件使能；
- 关闭窗口或 `Ctrl+C`：触发 `onCleanup`，关闭窗口并断开两个只读连接。

三幅图分别显示死区后的控制力、导纳/QP 的归一化广义速度、当前/预测
RCM 误差与候选硬边界。中间图和文字中 `u` 的顺序是：

```text
[pivot1 (deg/s), pivot2 (deg/s), insertion (mm/s)]
```

如果下方 RCM 图的当前误差已大于 2.0 mm，QP 会按设计拒绝预测并归零。这表明当前
机器人姿态、工具几何或所给 RCM 坐标不一致，需先检查，不应放大边界绕过。

### 2. 10 分钟正式只读记录

短时预检中 RCM 几何、反馈新鲜度和 QP 状态正常后运行：

```matlab
[summary, resultDir] = run_stage07_live_readonly_dryrun( ...
    DurationSec=600, ...
    FaultInjectionProfile="none");
```

在这次记录中，用 `SPACE` 锁存预测使能（或按住鼠标区域），并施加六组短时、可控的正负力：

1. 工具 `+Z/-Z` 轴向力：`insertion` 应以相反符号响应；
2. 工具 `+X/-X` 横向力：主要激发 `pivot2`，正负方向应反号；
3. 工具 `+Y/-Y` 横向力：主要激发 `pivot1`，正负方向应反号。

对当前 `ShaftAxisEndoscope=[0;0;1]` 和 RCM 位于尖端后方的正常几何，预期主符号约定是
`+Fx -> +pivot2`、`+Fy -> -pivot1`、`+Fz -> +insertion`。以实际工具轴标识和图中方向为准，
若符号不符，先保留日志并检查工具坐标/轴向，不改控制增益掩盖。

### 3. 故障注入记录

```matlab
[faults, faultDir] = run_stage07_live_readonly_dryrun( ...
    DurationSec=90, ...
    FaultInjectionProfile="acceptance");
```

程序在 15/30/45/60/75 s 分别模拟软件使能时间戳过期、力信号过期、机器人反馈过期、
QP 失败和控制周期超时。启动时命令行会打印完整时间表；每项注入前 5 s，图窗标题、
底部彩色提示区和命令行会同时显示下一项故障与倒计时。

不需要提前记住何时使能。看到黄色“准备”提示后：

1. 按一次 `Space`，确认区域进入绿色 `ENABLED`；
2. 持续施加约 3--5 N、方向稳定的外力，使注入前预测速度明确非零；
3. 到达 `T=15/30/45/60/75 s` 时，界面应进入红色 `FAULT`，预测速度归零；
4. 随后撤力，按 `Esc`，等待至少 0.5 s 让注入结束，再按 `R` 清除锁存故障，确认界面变蓝；
5. 保持 OFF，等下一项的黄色 `T-5 s` 提示后，再执行 `Space + 施力`。

所有倒计时采用 worker 的控制时钟，而不是 MATLAB 客户端启动或机器人连接所花的时间。
因此并行池预热、连接速度和图形刷新不会使提示与实际注入错位。`summary.json` 会记录
`FaultGuidanceEnabled` 和统一的 `FaultSchedule`；控制快照还会显示实际
`InjectionStatusCode`。

## 输出

每次运行保存到 `results/stage07_live_readonly_<timestamp>/`：

- `force_samples.csv`：原始力、fast/slow、死区后控制力、时序、质量位和基线状态；
- `control_predictions.csv`：FSM、全部安全位、QP、RCM 误差、导纳广义速度、预测关节速度/目标；
- `summary.json`：实际采样率、deadline miss、反馈 age、完整读取时间和 QP P50/P95/P99；
- `config_snapshot.json`、`sensor_config_snapshot.json`、`rcm_point_record.json`：可复现配置快照。

## 退出条件

需要同时满足才能将阶段 7 标记为“已通过”：

- 10 分钟正式记录完成，无非法数值或未解释的控制许可；
- `RobotCommandAttemptCount=0` 且 `RobotCommandSentCount=0`；
- 采样率、反馈 age、deadline miss 和 QP P99 有记录；
- pivot1/pivot2/insertion 的正负预测方向完成人工确认；
- 五种故障注入都让预测命令在预期周期内归零；
- 同步日志中包含当前/预测 RCM 误差和全部安全位。

该阶段通过后仍不会自动解锁真实运动；阶段 3 的工具/RCM 验证和阶段 8 的物理安全条件
需要另行完成。
