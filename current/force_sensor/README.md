# OnRobot HEX-H QC MATLAB 读取与可视化

该模块通过 Compute Box 的 Modbus TCP 接口读取 HEX-H QC 六维力/力矩，不依赖网页可视化页面。

## 通信参数

| 参数 | 值 |
|---|---:|
| Compute Box IP | `192.168.50.201` |
| Modbus TCP 端口 | `502` |
| HEX-E/H QC Unit ID | `64 (0x40)` |
| Fx～Tz 协议寄存器 | `259～264 (0x0103～0x0108)` |
| 力数据 | signed INT16，`0.1 N/count` |
| 力矩数据 | signed INT16，`0.01 N·m/count` |

注意：OnRobot 手册给出的是 Modbus 协议零基地址；MATLAB `modbus/read` 使用一基地址并在内部减一。本模块已经统一处理 `+1`，调用方不要再次偏移地址。

## 网络设置

1. 电脑有线网卡设置为同一子网内未占用的静态地址，例如 `192.168.50.200`；
2. 子网掩码设置为 `255.255.255.0`；
3. 确认浏览器仍能打开 `http://192.168.50.201`；
4. 关闭其他可能占用 Modbus 的程序。Compute Box 手册给出的并发连接数为 1；
5. 防火墙需要允许 MATLAB 访问 TCP 端口 502。

## 快速运行

在 MATLAB 中：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current/force_sensor/examples');
run_hex_h_live_monitor;
```

程序会：

- 读取 `Fx, Fy, Fz, Tx, Ty, Tz`；
- 显示最近 10 秒的实时曲线和当前数值；
- 在当前目录保存带时间戳的 CSV；
- 关闭绘图窗口后安全结束。

## 安全代码中的复用

安全控制循环只依赖传感器客户端，不需要绘图器：

```matlab
addpath(fullfile('/path/to/scopeguide-rcm-admittance-control/current', 'force_sensor'));

cfg = onrobot.defaultConfig();
sensor = onrobot.HexHClient(cfg);
sensor.connect();
cleanup = onCleanup(@() sensor.disconnect());

while robotIsRunning
    sample = sensor.readSample();

    F_sensor = sample.force;       % 3x1, N
    T_sensor = sample.torque;      % 3x1, N*m
    wrench = sample.wrench;        % 6x1, [F; T]
    sensorTime = sample.monotonicTime;

    % compensatedWrench = compensateWrench(wrench, robotState, calibration);
    % safetyCommand = safetyStateMachine(compensatedWrench, visualRisk, ...);
end
```

## ScopeGuide 阶段 1 离线处理链路

项目根目录中的 `+scopeguide/+force/ForceProcessingPipeline.m` 在本目录已有客户端和
滤波器之上补齐了控制级离线链路：

```text
样本字段/有限值/序号/时间戳/状态检查
→ 静态零偏与姿态重力补偿
→ 受严格门控的可选残余基线
→ 20 Hz 快支路 + 5 Hz 控制支路
→ 保持方向的连续矢量死区
→ raw/fast 安全判断
```

完整重力补偿重放必须使用同时记录了机器人四元数的数据。运行：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');
[replay, outputDirectory] = run_stage01_force_replay();
```

默认读取当前力觉标定批次的 `raw_samples.csv`，只重放当前标定保留的姿态 1–8、10，
并在 `results/stage01_force_replay_*` 保存配置、摘要、MAT 和 CSV。

早期的 `hex_h_static_*.csv` 和 `hex_h_contact_*.csv` 没有机器人姿态，只能通过
`scopeguide.force.replayPoseFreeRawDataset` 验证原始数据和滤波支路。该函数明确返回
`CompensationApplied=false` 和 `ControlEligible=false`，禁止把未补偿数据送入导纳。

第一版只输出 `ControlForceTool`。补偿后力矩仍关于传感器测量原点，仅作诊断与安全检查；
在 `p_S^E` 标定完成前，`MomentEligibleForControl` 始终为 `false`。

## ScopeGuide 完整处理结果实时显示

阶段 2 的机器人只读反馈可用后，可以同时读取 Nova5 姿态和 HEX-H，实时显示完整处理
链路的最终结果：

实时获取和处理的唯一生产接口是：

```text
+scopeguide/+io/ForceSensorAdapter.m
```

它内部直接调用 `onrobot.HexHClient` 和 `scopeguide.force.ForceProcessingPipeline`。监视器
只调用 `ForceSensorAdapter.readProcessed()` 并显示返回的 `Processed`，不包含第二套补偿、
滤波或死区实现；以后主控制循环也应调用同一个接口。

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');
clear functions;
[stats, outputDirectory] = run_processed_force_live_monitor( ...
    DurationSec=60, WindowSec=10, DisplaySignal="slow");
disp(stats);
disp(outputDirectory);
```

也可以不限制时间，关闭绘图窗口时结束：

```matlab
[stats, outputDirectory] = run_processed_force_live_monitor();
```

`DisplaySignal="slow"` 显示的是重力/零偏补偿、软件基线和 5 Hz 滤波后的六轴结果，
不经过力/力矩死区。`DisplaySignal="control"` 才显示死区后的 `ControlForceTool`；软件
操作使能关闭时，该控制力会被强制为零。下图的 `Tx/Ty/Tz` 仍关于 HEX-H 测量原点，
不能进入控制。

该监视器：

- 六轴寄存器按配置的 200 Hz 目标轮询，设备状态寄存器独立按 20 Hz 轮询；
- 使用当前 Nova5 四元数做姿态相关重力补偿；
- 启动时要求连续静置、无人工接触 3 s，并对补偿后的六轴残差执行一次软件置零；
- 基线静止判定默认允许关节反馈抖动不超过 2 deg/s、TCP平移不超过 3 mm/s、
  TCP转动不超过 2 deg/s；这些只用于基线门控，不是机械臂运动安全阈值；
- 空闲且通过静止/无接触残差门控时，只以 30 s 时间常数缓慢跟踪 Fz；
- 点击图窗右上角“操作使能”或按 `E` 后冻结基线并允许控制力输出，按 `Esc` 关闭；
- 显示 quality、raw/fast safety 和机器人反馈 age；
- 默认保存 raw、补偿后、fast、slow 和最终信号到
  `results/processed_force_live_<timestamp>/signals.csv`；
- 不调用 HEX-H 硬件 `zero/unzero`；
- 不调用机器人 `Enable/ServoJ/MovJ/MovL`。

绘图默认只按 10 Hz 刷新，不降低 CSV 中六轴数据的目标采样率。状态寄存器采用缓存值的
最长时间约 50 ms；正式运动控制前仍需评估是否应通过单次 Modbus 批量读取进一步缩短状态
检测延迟。

如果机器人反馈无效或过期，完整链路会 fail closed，控制力输出为零。公共配置中的自动
基线仍默认关闭，但该实时 monitor 默认通过 `EnableAdaptiveFzBaseline=true` 显式开启上述
策略。需要观察完全不经过软件基线的数据时，可设置 `EnableAdaptiveFzBaseline=false` 并
使用 `DisplaySignal="external"`。

这里的“操作使能”只是一枚软件状态开关，用于冻结基线和放行 `ControlForceTool`，不会使能
机械臂。它不能替代实体急停、安全回路或需要持续按住的硬件 deadman；正式驱动机械臂时应
接入硬件使能信号，并让释放信号直接触发停止。

也可以向实时监视器传入回调：

```matlab
onrobot.monitor(sensor, 'RateHz', 200, ...
    'SampleCallback', @(sample) safetyInputQueue(sample));
```

## 测试最高稳定读取频率

该测试不创建可视化窗口，也不会修改传感器零偏：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current/force_sensor/examples');
report = run_hex_h_frequency_benchmark;
```

程序首先连续读取 100 次以估计吞吐上限，然后自动扫描最高 500 Hz 的候选频率。每个频率默认测试 5 秒，并报告：

- 实际达到的频率；
- 单次读取耗时的平均值、P95 和 P99；
- 采样周期绝对抖动 P95；
- deadline miss 比例；
- 是否满足稳定性判据。

默认稳定性判据为：实际频率不低于目标的 98%、deadline miss 不超过 1%、周期抖动 P95 不超过目标周期的 25%。结果保存为 CSV 和 MAT。

这里测量的是 PC 通过 Modbus TCP 的稳定轮询频率。轮询频率高于传感器内部刷新频率时可能重复读到同一帧，因此该结果不能直接解释为传感器物理测量带宽。

2026-08-07 当前 PC、网络和 Compute Box 的实测结果：

- 200 Hz：实际 200 Hz，P95/P99 读取耗时 2.628/3.040 ms，deadline miss 为 0；
- 350 Hz：当前判据下最高稳定测试频率，deadline miss 约 0.057%；
- 400 Hz：deadline miss 约 8.9%，判为不稳定；
- 完整报告：`examples/hex_h_frequency_20260807_161931.csv/.mat`。

因此完整处理链路继续使用 200 Hz，不使用 350 Hz；后者只是当前单独 Modbus 轮询的性能
上界，没有包含机器人姿态同步、处理、日志和控制计算预算。

如需指定测试频率：

```matlab
report = onrobot.benchmarkFrequency(sensor, ...
    'RatesHz', [25 50 75 100 125], ...
    'DurationPerRateSec', 10);
```

## 实时滤波

基于固定负载 200 Hz 静态数据，推荐保留两条因果滤波支路：

- 快速支路：20 Hz EMA，用于冲击、快速力上升和硬保护；
- 慢速支路：二阶 5 Hz Butterworth，用于持续压迫、方向估计和释放判断。

```matlab
filters = onrobot.WrenchFilterBank(200, 20, 5);

while robotIsRunning
    sample = sensor.readSample();
    [wrenchFast, wrenchSlow] = filters.step(sample.wrench);

    % wrenchFast -> impact / dFdt / hard protection
    % wrenchSlow -> sustained contact / direction / release
end
```

该实现逐样本运行且不依赖 Signal Processing Toolbox。当前数据估计的快速支路延迟约 5.7 ms，慢速支路延迟约 45 ms。低通滤波不会消除重力、安装偏置或慢漂移；这些应通过负载补偿和仅在确认无接触时更新的基线估计器处理。

固定负载数据可以通过以下命令重新记录和分析：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current/force_sensor/examples');
[data, stats] = record_static_load(60);  % 200 Hz，60 秒，无绘图
report = analyze_latest_static_load();
```

原始数据、统计结果、滤波器比较和完整分析分别保存在 `data/` 下的 CSV/MAT 文件中。

## 后端选择

- `cfg.Backend = 'modbus'`：使用 MathWorks `modbus`，需要 Industrial Communication Toolbox；R2022a 以前该功能属于 Instrument Control Toolbox。
- `cfg.Backend = 'tcpclient'`：模块自行构造 Modbus TCP 报文，只要求 MATLAB 提供 `tcpclient`。
- `cfg.Backend = 'auto'`：优先使用 `modbus`，失败时回退到 `tcpclient`。

## 零偏操作

```matlab
sensor.zero();    % 将当前读数设为零偏
sensor.unzero();  % 恢复默认零偏
```

代码不会自动执行 `zero()`。只有在传感器无外部接触、安装状态稳定，并且明确希望修改设备偏置时才能调用。后续重力/负载补偿不应由频繁设备清零代替。

## 测试

无需连接传感器即可测试 signed INT16 和量纲缩放：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current/force_sensor');
results = runtests('tests');
table(results)
```

## “端口可连接但读取超时”的排查

如果 TCP 502 可以建立连接，但程序提示等待 Modbus 报文超时，说明网络路由基本正常，但 Compute Box 没有响应当前请求。OnRobot 手册明确说明：设置不正确时设备可能完全不返回 Modbus 异常报文。依次检查：

1. 关闭机器人控制器、PLC、Modbus Poll 等可能已经连接 Compute Box 的客户端；官方给出的最大并发连接数是 1；
2. 关闭网页实时监视页面后重试，避免当前固件的内部监视任务占用资源；
3. 在网页中确认 HEX-H 已被 Compute Box 正确识别且状态正常；
4. 确认使用 `Unit ID = 64`，不要把 Compute Box 自身的 `63` 当作 HEX-H 地址；
5. 确认当前 Compute Box/机器人配置允许 Modbus TCP，而不是只启用了 EtherNet/IP 或机器人专用接口；
6. 若之前存在异常断开的 Modbus 客户端，关闭相关程序并重启 Compute Box，再运行 MATLAB 示例；
7. 将 `cfg.Backend` 分别设为 `'modbus'` 和 `'tcpclient'`。如果两者均超时，问题位于 Compute Box 配置/占用，而不是 MATLAB 报文解析。

仅仅看到端口 502 处于 open 状态并不代表寄存器读取已经可用；必须成功读取 `259～264` 才算完成端到端验证。

## 资料

- [OnRobot 用户手册中的 Modbus TCP 参数与 HEX-E/H QC 寄存器表](https://onrobot.com/sites/default/files/documents/User_Manual_for_TECHMAN_OMRON_TM_v1.1.2_EN_0.pdf)
- [MathWorks Modbus read 文档](https://www.mathworks.com/help/icomm/ug/modbus.read.html)
