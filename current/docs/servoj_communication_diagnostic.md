# ServoJ 30003 通讯隔离诊断

该程序只检查 Dobot 30003 端口的 `ServoJ` 发送与回复行为，不连接
HEX-H，不运行重力补偿、RCM、QP、导纳控制，也不会调用
`EnableRobot`/`DisableRobot`。

每组试验开始前从 30004 读取实际关节角，并在整组试验中重复发送这个固定
关节角。因此程序不主动请求位移，但它仍然属于物理 `ServoJ` 下发：必须使用
机械夹具，确保净空，并让示教器与急停始终可触及。该程序没有力传感器安全
保护，测试期间不要推拉末端。

## 对照内容

- `scopeguide_minimal`：修正前 Stage 8 的 `ServoJ(q)`，用于复现问题。
- `legacy_parameterized`：当前修正后的参数化格式，`t=1/测试频率`，
  `lookahead_time=60`、`gain=400`；在 25 Hz 时也精确复现旧项目的
  `t=0.04`。
- `continuous_framed`：持续读取、重组并解析分包/粘包回复。
- `legacy_immediate`：复现旧项目写入后只检查一次
  `NumBytesAvailable` 的行为。
- 默认分别以 20 Hz 和旧项目的 25 Hz 测试。

每组使用一条新建的 30003 TCP 连接，防止前一组未处理完的回复污染后一组。
图形只在采集完成后生成，不会干扰发送时序。

## 启动命令

先在 DobotStudio/示教器中进入 TCP/IP 控制模式并使能机器人，然后运行：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');

[servoSummary, servoDir] = run_servoj_communication_diagnostic( ...
    ArmPhysicalMotion=true, ...
    MotionConfirmation="RUN_SERVOJ_HOLD_DIAGNOSTIC", ...
    FixtureAndClearanceConfirmed=true, ...
    SecondObserverPresent=true);
```

程序默认执行八组短测试，总耗时约 50 秒。过程中无需按 Space；如发现任何
非预期运动，立即使用示教器/急停中止。

## 输出

- `commands.csv`：每条命令的发送周期、写入耗时、回复延迟、ErrorID 和原文。
- `read_chunks.csv`：TCP 每次读取的字节数、完整回复数和残包长度。
- `trials.csv`：每组的发送/回复吞吐率、延迟斜率和最大积压。
- `summary.json`：自动对比旧、新实现并给出诊断结论。
- `servoj_communication_diagnostic.png`：发送周期、回复延迟和积压图。

判断重点：

- 若旧参数格式正常而 `ServoJ(q)` 持续积压，问题主要来自命令参数差异。
- 若两种格式都积压，30003 实际回复/处理吞吐低于请求下发频率。
- 若两种读取方式只改变观测延迟、不改变最终回复数量，读取方式不是根因。
- 旧项目允许空回复且没有启用 ErrorID 解析，因此“旧项目正常运行”并不等价于
  “每条命令都在一个控制周期内得到了确认”。
