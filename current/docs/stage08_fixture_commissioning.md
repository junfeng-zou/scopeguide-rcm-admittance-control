# Stage 8：机械 RCM 夹具低速 ServoJ 调试

## 当前实现

- 控制/通信在独立 MATLAB process worker 中以 20 Hz 运行，前台默认以 10 Hz 绘图；Stage 7 与离线默认配置仍保持 30 Hz；
- `RobotAdapter` 是唯一 `ServoJ` 发送边界，每条命令前复核授权、机器人模式、反馈、关节限位和单步目标；
- 每条 `ServoJ` 显式发送 `t=1/ControlRateHz`、`lookahead_time=60`、`gain=400`；当前 20 Hz 调试配置对应 `t=0.05 s`；
- Stage 8 直接调用用户在 `dobot_HW_control2` 中验证过的 `ZJFDobotCR5.ServoJ`，使用原始 `sprintf` 完整参数格式；`sendMoveCommand` 也整体恢复为旧项目行为：写入后只立即检查一次可用字节，有数据就读取、没有就返回空字符串，不检查发送前缓冲区、不等待分号完整回复，也不使用逐条回复超时；
- 软件使能沿用 `Space -> ON`、`Esc -> OFF` 的锁存方式，输入心跳超时为 250 ms；
- 软件键盘不是物理 deadman。通过全部阶段 8 授权门后，后台会执行 `EnableRobot(1.3)`、`SpeedFactor(20)` 并等待 2 s，再以 30004 的机器人模式、反馈 age 和 `SpeedRatio` 确认初始化；退出时执行 `DisableRobot()`；
- CR5 绝对关节范围采用制造商规格：J1/J2/J4/J5/J6 为 ±360°，J3 为 ±160°，QP 和命令层再保留 5°；
- 所有任务速度、关节速度和加速度均乘以
  `cfg.stage08.SpeedScale`（实际值以配置文件为准，允许范围为
  `(0, 4.5]`）；
- 当前 `cfg.stage08.SpeedScale=2.5`，即 Stage 8 基准参数的 250%；
- pivot1 与 pivot2 共用相同的 Stage 8 调参：全局速度/安全上限保持120%，pivot导纳手感尺度独立设为160%；5 N设计输入对应的目标稳态角速度为 `2.133 deg/s`，任务速度上限仍为 `3.0 deg/s`。因此达到同一未饱和速度所需的广义力约下降25%，且不改变 insertion、关节速度和关节加速度限制；
- HEX-H 测量原点按法兰 `+Z 20 mm` 写入 `cfg.tool.TFlangeSensor`。控制输入使用完整六维 wrench，先按
  `M_RCM=M_sensor+(p_sensor-p_RCM)×F` 平移到 RCM，再将两个横向力矩映射为 pivot1/pivot2；
  insertion 仍取沿镜杆轴的力，roll 仍关闭；
- 默认只开放 insertion，roll 始终关闭；
- 默认停止阈值为 raw force 60 N、fast force 40 N、raw moment 4 N·m、fast moment 2 N·m；本轮仍要求逐次输入明确确认短语；
- 每次运行保存同步控制日志、ServoJ 写入及立即读取耗时、可选回复计数、目标—反馈误差、模型 RCM 误差和松开停止延迟；窗口第四幅图及 CSV 同时记录六轴指令/实际关节速度。
- 力故障保留原始原因：数据质量问题显示 `FORCE_*`，阈值触发直接显示 `FAST_FORCE_STOP`、`RAW_FORCE_STOP`、`FAST_MOMENT_STOP` 等安全监视器原因。
- `0.30 deg` ServoJ 步长保护用于检查新目标相对上一条成功命令的离散跳变；在 20 Hz 下对应名义 `6 deg/s` 的目标增量边界。它是超过后触发 Fault 的离散跳变检查，并非速度限幅。首条命令相对最新反馈检查，通信与伺服跟踪滞后由独立的 `0.50 deg`、连续 3 个控制周期保护处理。
- Stage 8 正常结束、故障转入监视或 Ctrl+C 清理后，默认冻结并保留最终完整时间轴曲线；键盘和鼠标控制回调会被移除，图窗只用于分析，由操作者手动关闭。若确实不希望保留，可显式传入 `KeepFigureOpenAfterRun=false`。

## 运行前硬件条件

只在机械孔板、球铰或明确套管夹具的自由空间完成，不在组织、人体或鼻腔 phantom 中进行。每次运行前同时满足：

1. 独立急停经过人工确认，并保持可触及；
2. 第二名观察者在场；
3. 机械臂和内窥镜周围有明确退出空间；
4. RCM 夹具、工具安装、末端负载和传感器标定均未改变；
5. 将 CR5 切换到 TCP/IP 控制模式；不要求预先从示教器使能，程序会进行软件使能，但示教器和急停必须始终可触及；
6. 首次只运行 `insertion_only`，不得直接跳到 pivot 或 3-DOF。

## 0. 仅连接、保持当前位置

`hold` 模式不会产生 ServoJ 目标，用于检查连接、软件置零、状态显示和 `Esc`：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');

[holdSummary, holdDir] = run_stage08_fixture_commissioning( ...
    DurationSec=15, ...
    DofMode="hold", ...
    ArmPhysicalMotion=true, ...
    MotionConfirmation="ENABLE_SCOPEGUIDE_MOTION", ...
    SetupConfirmation="STAGE8_FIXTURE_READY", ...
    SafetyThresholdConfirmation="CONFIRM_LIMITS_60N_40N_4NM_2NM", ...
    SecondObserverPresent=true, ...
    FixtureAndClearanceConfirmed=true);
```

## 1. insertion-only 首次低速运动

```matlab
[insSummary, insDir] = run_stage08_fixture_commissioning( ...
    DurationSec=60, ...
    WindowSec=60, ...
    PlotRateHz=10, ...
    DofMode="insertion_only", ...
    ArmPhysicalMotion=true, ...
    MotionConfirmation="ENABLE_SCOPEGUIDE_MOTION", ...
    SetupConfirmation="STAGE8_FIXTURE_READY", ...
    SafetyThresholdConfirmation="CONFIRM_LIMITS_60N_40N_4NM_2NM", ...
    SecondObserverPresent=true, ...
    FixtureAndClearanceConfirmed=true);
```

操作顺序：

1. 软件置零期间不施力、不按 Space；
2. 显示区变为可用后按一次 Space；
3. 先施加很小的正向 insertion 力，撤力；再施加反向力；
4. 主动按 Esc，确认立即停止并保持蓝色；
5. 至少完成一次 `Space -> 运动 -> Esc -> 停止`，否则无法计算停机延迟；
6. 用机械标记或量具记录真实镜杆在 RCM 孔处的最大横向位移，不能只看图中的模型 RCM；
7. 若出现方向错误、持续运动、振荡、追赶或急停动作，直接判失败，不接受该结果。

程序返回后曲线窗口仍会保留，而且横轴会从滑动窗口自动展开到本次运行的完整时长。此时 Space、Esc、R 和鼠标使能区域均已失效，只能查看、缩放、保存或手动关闭图窗。

## ServoJ 独立微动诊断

如果 Stage 8 已经发送目标、但机械臂肉眼看起来完全不动，先运行与力、QP、RCM 和积分器完全隔离的单关节微动诊断。它默认让 J4 用 1 s 平滑移动 `+0.10 deg`、保持 1 s，再用 1 s 回到起点；前后各保持 1 s。运动很小，主要依据 30004 反馈曲线和自动结论判断，不能只靠肉眼。

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');

[microSummary, microDir] = run_servoj_micro_motion_diagnostic( ...
    JointIndex=4, ...
    OffsetDeg=0.10, ...
    ArmPhysicalMotion=true, ...
    MotionConfirmation="RUN_SERVOJ_MICRO_MOTION_DIAGNOSTIC", ...
    ProgrammaticEnableConfirmation= ...
        "ENABLE_ROBOT_FROM_MICRO_DIAGNOSTIC", ...
    RobotInTcpModeConfirmed=true, ...
    FixtureAndClearanceConfirmed=true, ...
    SecondObserverPresent=true);
```

运行前选择好 TCP/IP 控制模式并保持急停可触及；程序不限制初始使能状态，机械臂处于 `DISABLED` 或已经 `ENABLE` 均可。满足全部确认门后，程序会通过 `dobot_HW_control2` 原版 `ZJFDobotCR5` 依次调用 `EnableRobot(1.3)`、`SpeedFactor(20)`、等待 2 s，再显示 3 s 倒计时并自动执行一次往返；默认在退出清理时调用 `DisableRobot()`。该程序不使用 Space。结果中的 `PASS` 表示实际关节反馈已明显跟随并回到起点；`NO_MEASURABLE_MOTION` 表示 30003 虽已发送目标，但 30004 没有测得相应运动；`PARTIAL_TRACKING` 表示有运动但幅值不足。最终诊断图默认保留，手动关闭即可。

当前 `ZJFDobotCR5` 保留了原版 Dashboard/ServoJ 立即读取、不阻塞等待逐条回复的发送行为，同时补回了 ScopeGuide 所需的反馈时间戳、反馈序号、四元数、`RunQueuedCmd` 和 `PauseCmdFlag`。独立微动诊断仍只以关节反馈跟踪作为通过依据，阶段 8 则使用完整反馈快照进行安全检查。

人工检查无误后才生成下一阶段配置。例如实测最大横向偏差为 0.8 mm：

```matlab
[cfgPivot1, insAcceptance] = accept_stage08_fixture_result( ...
    insDir, ...
    DirectionCorrect=true, ...
    PhysicalRcmWithinHardBound=true, ...
    PhysicalRcmMaximumMm=0.8, ...
    ReleaseStopWasImmediate=true, ...
    NoUnexpectedMotion=true, ...
    Confirmation="ACCEPT_STAGE8_PHASE");
```

只有 insertion 的命令发送时序、反馈跟踪误差、模型 RCM、实测停止延迟及人工检查全部通过，生成的 `cfgPivot1` 才会包含 `ServoTimingVerified=true`；30003 可选回复的有无不参与通过判定。

## 2. 后续自由度顺序

每次都使用上一次 `accept_stage08_fixture_result` 返回的配置：

```matlab
[p1Summary, p1Dir] = run_stage08_fixture_commissioning( ...
    Config=cfgPivot1, DurationSec=60, DofMode="pivot1_only", ...
    ArmPhysicalMotion=true, ...
    MotionConfirmation="ENABLE_SCOPEGUIDE_MOTION", ...
    SetupConfirmation="STAGE8_FIXTURE_READY", ...
    SafetyThresholdConfirmation="CONFIRM_LIMITS_60N_40N_4NM_2NM", ...
    SecondObserverPresent=true, FixtureAndClearanceConfirmed=true);
```

对 `p1Dir` 完成同样的人工接受，取得新配置后，严格按以下顺序继续：

1. `insertion_only`
2. `pivot1_only`
3. `pivot2_only`
4. `dual_pivot`
5. `full_3dof`

代码会拒绝跳级。每个阶段均需正/反方向、Esc 停止、实测 RCM 和无意外运动检查。

## 3. 阶段 8 最终复核

五个结果目录全部接受后：

```matlab
dirs = [insDir; p1Dir; p2Dir; dualDir; fullDir];
[review8, reviewDir8] = run_stage08_final_review(dirs);
```

仅当输出 `Stage 8 final review: PASSED_FIXTURE` 时，阶段 8 才算通过。该状态只适用于当前未改动的机械夹具，不能直接授权阶段 9 的鼻腔 phantom。

## 当前 dual-pivot 短期固定参数

`current_stage08_dual_pivot_config()` 固定了 2026-08-17 人工确认可用的短期参数，并自动加载已经验收的 pivot2 前置配置：pivot 阻尼及 `2.133 deg/s` 设计目标不变，虚拟质量尺度为 `40%`（时间常数 `0.12 s`），合成 pivot 范围为 `7.5 deg`，RCM 保持 `soft` 且硬边界保持 `2 mm`。该函数不填写或绕过任何物理运动安全确认项。

`current_stage08_full_3dof_config()` 是可人工编辑的 pivot + insertion 参数入口。它默认加载已经验收的 dual-pivot 配置以保留阶段前置证据、硬件几何和标定记录，并显式列出最近一次 full-3DOF 运行的最终参数。当前 insertion 仍保持 `5 N -> 1.2 mm/s`、时间常数 `0.30 s`、最大速度 `2.4 mm/s`、范围 `+/-8 mm`；此前讨论的 `2.0 mm/s` 降阻尼候选及扩大插入范围均未写入。该函数的数值已经是 Stage 8 最终值，`ProfileApplied=true` 会阻止运行入口再次应用速度缩放。

## 立即停止条件

- 按 Esc 后仍持续运动；
- RCM 孔处的横向位移超过当前 2.0 mm 硬限制；
- 出现 `SERVO_COMMAND_EXCEPTION`、`HOLD_COMMAND_EXCEPTION`、feedback stale、QP fault 或 tracking error；
- 目标—反馈误差持续增加；
- 机械臂进入非 `ENABLE/RUNNING` 状态；
- 任一电缆受拉、夹具松动、末端负载或工具几何发生变化。

出现上述任一项时不要通过增加阻尼掩盖问题，应退回 Stage 7 只读测试或重新标定。
