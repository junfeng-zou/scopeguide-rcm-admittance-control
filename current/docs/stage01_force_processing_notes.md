# 阶段 1：力觉处理链路实现说明

## 1. 当前实现范围

阶段 1 已实现纯离线、确定性的力觉处理链路：

```text
ForceSample 数据质量检查
→ 标定零偏和姿态重力补偿
→ 传感器坐标旋转到工具轴表达（原点仍是传感器测量中心）
→ 默认关闭、受严格条件门控的残余基线估计
→ 20 Hz 快支路和 5 Hz 慢支路
→ 保持方向的连续矢量死区
→ raw / fast 安全判断和 fast + slow neutral check
→ 关于 HEX-H 原点的六维控制 wrench 与完整重放日志
```

该阶段不连接机器人和力传感器，也不发送任何运动命令。

## 2. 数据使用边界

完整重力补偿需要每个 wrench 对应的机器人姿态。当前可用于完整重放的数据是：

```text
force_calibration/data/hex_h_tool_calibration_20260807_164630/raw_samples.csv
```

该文件包含 10 个姿态、每姿态 800 点。当前采用的
`calibration_without_pose09.json` 只保留姿态 1–8、10，因此默认重放 7200 点。

以下早期记录没有机器人四元数：

```text
force_sensor/data/hex_h_static_20260718_202428.csv
force_sensor/data/hex_h_contact_20260718_210529.csv
```

它们只能验证原始 wrench 和滤波器，不能严格做姿态重力补偿。对应重放函数会明确返回：

```text
CompensationApplied = false
ControlEligible = false
```

## 3. 当前阈值依据

| 数据/参数 | 当前值 | 说明 |
|---|---:|---|
| 标定记录最大 raw 合力 | `17.697 N` | 新批次 10 姿态、8000 点 |
| 标定记录最大 raw 合力矩 | `1.235 Nm` | 力矩关于传感器原点 |
| 旧 static 最大 raw 合力 | `11.917 N` | 无姿态，不能补偿 |
| 旧 contact 最大 raw 合力 | `19.535 N` | 无姿态，不能补偿 |
| 当前验证最大慢速残余合力 | `1.177 N` | 新九姿态 7200 点完整回放 |
| force deadzone | `1.3 N` | 台架起点，不是最终阈值 |
| neutral force limit | `1.3 N` | 同时检查 fast 和 slow |
| moment deadzone / neutral | `0.18 Nm` | pivot 使用，roll 仍关闭 |
| raw force/moment stop | `60 N / 4 Nm` | 当前 Stage 8 确认值 |
| fast warning/stop | `6 N / 40 N` | 当前 Stage 8 确认值 |

这些阈值没有临床意义。配置中的 `cfg.safety.ThresholdsValidated` 和
`cfg.force.ParametersValidated` 仍为 `false`，因此物理运动许可门不会因为阶段 1 完成而开放。

## 4. 基线策略

- 默认 `AutomaticBaselineEnabled=false`；
- 只有外部明确给出“握把未使能、机器人静止、已排除接触、允许更新”四个条件时才可更新；
- 任一条件失效就停止学习，但不会删除已经估计的基线；
- 更新速度和总偏移均有限制；
- 使能期间持续外力不会被学习为零偏。

阶段 4 的 deadman 状态机完成前，不建议在任何联机路径开启自动基线。

## 5. 力矩与参考点（2026-08-16 更新）

当前补偿后的力矩已经旋转到工具/法兰轴方向，但原点仍是 HEX-H 测量中心。现按用户给出的
几何估计设置 `TFlangeSensor` 平移为法兰 `+Z 20 mm`，控制层采用
`M_RCM = M_sensor + (p_sensor-p_RCM) × F` 将完整 wrench 平移到 RCM：

- `ControlWrenchToolAtSensorOrigin` 是进入导纳的六维量；
- pivot1/pivot2 使用平移到 RCM 后的力矩；
- insertion 仍使用沿镜杆轴的力；
- `ControlMomentForDiagnostics` 继续保留给图形和诊断；
- roll 仍严格关闭。

## 6. 运行方法

运行全部离线测试：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');
clear functions;
results = runAllTests();
disp(table(results));
assert(all([results.Passed]));
```

运行完整阶段 1 重放并保存结果：

```matlab
[replay, outputDirectory] = run_stage01_force_replay();
disp(replay.Summary);
disp(outputDirectory);
```

输出目录为：

```text
results/stage01_force_replay_<timestamp>/
├── summary.json
├── config_snapshot.json
├── signals.mat
└── signals.csv
```

只有测试全部通过、完整重放无非法样本且结果与当前验证残余一致后，才能把阶段 1 标记为
“已通过”。

## 7. 2026-08-07 验收结果

结果目录：

```text
results/stage01_force_replay_20260807_015914_059/
```

验收结果：

| 指标 | 结果 |
|---|---:|
| 全部自动测试 | `28/28 Passed` |
| 完整重放样本 | `7200` |
| 有效/无效样本 | `7200 / 0` |
| 丢失样本 | `0` |
| safety warning / stop | `0 / 0` |
| neutral check 通过 | `7021 / 7200` |
| 最大补偿后瞬时合力 | `2.2834 N` |
| 最大慢速合力 | `1.4099 N` |
| 死区后最大控制力 | `0.1099 N` |
| 最大补偿后力矩 | `0.1740 Nm` |

死区后只有 `31/7200` 个样本非零，最长连续 `9` 个样本；按 200 Hz 计约 `45 ms`，不构成
持续漂移。超过 `0.05 N` 的样本有 15 个，超过 `0.1 N` 的样本有 3 个。

因此阶段 1 的离线退出条件已满足。这里的“已通过”只表示离线力觉链路通过；
`ParametersValidated` 和 `ThresholdsValidated` 仍保持 `false`，真实运动阈值必须在后续机械夹具
和 phantom 阶段验证。

2026-08-14 更新：用户根据最终负载的重力/零偏标定、20/5 Hz 滤波、启动软件置零、Fz
慢跟踪、1.3 N / 0.18 N*m 死区以及阶段 7 实机只读表现，确认当前力处理参数可用于
自由空间内窥镜微调 commissioning。因此默认配置将 `force.ParametersValidated=true`。
该确认不同时验证 raw/fast 停机阈值，`safety.ThresholdsValidated` 继续保持 `false`；装配或
力标定变化后必须撤销本确认。记录见 `config/stage08_user_attestation_20260814.json`。
