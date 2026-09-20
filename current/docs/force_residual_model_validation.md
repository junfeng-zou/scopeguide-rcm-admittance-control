# HEX-H 残差来源辨识与独立验证程序

入口程序：`run_force_residual_model_validation.m`

该程序实现残差处理顺序中的第 4–7 步：多姿态采集、四种误差假设比较、独立留出姿态验证，以及仅在全部判据通过后生成版本化的零偏候选文件。

## 实验设计

- 共使用 9 个不同静止姿态，每个姿态默认采集 20 s、200 Hz。
- 姿态 1、2、3、5、6、8 是训练姿态，只用于拟合。
- 姿态 4、7、9 是预先固定的留出验证姿态，不参与任何拟合。
- 第 10 次采集回到初始姿态 A，用于检查实验期间的漂移、线缆预载变化和重复性。
- 默认仅改变控制器法兰姿态，控制器 X/Y/Z 保持不变；程序会显示名义内窥镜尖端位置，但该预览不是碰撞检测。

程序比较以下四种模型：

| 模型 | 固定量 | 拟合量 | 用途 |
|---|---|---|---|
| Original | 当前全部标定参数 | 无 | 当前补偿效果基线 |
| BiasOnly | 当前质量与 CoG | 六轴常量零偏修正 | 检验残差是否主要是常量零偏 |
| LoadOnly | 当前六轴零偏 | 质量与 CoG | 检验负载参数是否发生变化 |
| Joint | 无 | 零偏、质量与 CoG | 检验联合重新标定能否显著改善 |

## 运行前提

1. 内窥镜必须完全离开患者、模型和狭窄通道。
2. 检查全部目标姿态和 `MovJ` 完整扫掠空间，而不只是端点。
3. 工具不接触任何物体，线缆在所有姿态下均松弛且无明显牵拉。
4. 机械臂已由操作者手动使能并处于空闲状态，急停可立即触及。
5. 不执行 HEX-H `ZERO` 或 `Auto-calibration`。
6. 本程序使用 Nova 5 V3 TCP/IP 接口以及现有 `ZJFDobotCR5.MovJ()`。

程序在第一次采集前检查 V3 反馈中的 `RunQueuedCmd` 和 `PauseCmdFlag`。如果运动队列未运行，程序会在任何采样或运动之前退出。此时不能直接发送 `Continue()`，因为控制器中可能保留上一轮已经接受但尚未执行的目标；应先在确认扫掠空间安全后通过控制器/DobotStudio执行 `ResetRobot` 清空旧队列，再由操作者重新使能机器人。

## 运行方法

先在 MATLAB 中执行：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');

[result, outputDir] = run_force_residual_model_validation( ...
    EnableMotion=true);
```

默认速度比例为 10%。如确有必要，可以显式设置为最高 20%：

```matlab
[result, outputDir] = run_force_residual_model_validation( ...
    EnableMotion=true, ...
    SpeedRatioPercent=20);
```

如果实验在某个姿态采集期间中断，且此前姿态已写入 `diagnostic.mat`、工具和线缆状态没有改变，可以从该结果目录断点续测。例如从已完成 P01–P07 的数据继续 P08：

```matlab
[result, outputDir] = run_force_residual_model_validation( ...
    EnableMotion=true, ...
    ResumeDirectory="/path/to/scopeguide-rcm-admittance-control/current/results/force_residual_model_validation_20260807_215752_693");
```

恢复模式强制沿用原始标定文件、姿态计划、采样参数和训练/验证划分。程序会检查当前反馈是否位于下一个目标；已经位于目标时不发送运动命令，否则仍需要确认后才发送 `MovJ`。恢复前如果上一轮触发了 `Pause()`，应先清空可能残留的队列、重新使能并确认 `RunQueuedCmd=1`。如果工具受力、线缆布置、传感器零状态或安装关系发生变化，则不能续测，应重新采集全部姿态。

程序首先打印十个端点。在输入总确认短语后，每次运动和每次采集仍需要单独输入对应短语。每次采集前都应重新确认工具无接触且线缆松弛。

确认短语必须与提示一致。例如，运动前输入 `MOVE P02_train`，到达并静止后再输入 `CAPTURE P02_train`。输错时程序不会下发命令，而是保留当前连接并要求重新输入；输入 `CANCEL` 才会安全停止并退出。

## 输出

结果保存在：

```text
results/force_residual_model_validation_时间戳/
```

主要文件：

- `raw_samples.csv`：每个采样点的原始六轴、当前补偿结果、机器人关节/笛卡尔反馈和四元数。
- `pose_summary.csv`：每姿态中值、当前模型残差和 BiasOnly 残差。
- `model_comparison.csv`：四种模型在训练集与留出验证集上的力/力矩误差。
- `residual_models.png`：各姿态残差范数对比。
- `diagnostic.mat`：完整数据、姿态摘要和判定对象。
- `summary.json`：不包含大矩阵的实验摘要。

如果 BiasOnly 的所有训练、留出验证和 A 回位判据均通过，还会生成：

```text
candidate_bias_only_calibration_时间戳.json
candidate_bias_only_calibration_时间戳.mat
```

候选文件明确标记为 `VALIDATED_CANDIDATE_NOT_ACTIVATED`，且 `activationAuthorized=false`。程序不会修改源标定文件，也不会修改 `defaultRcmAdmittanceConfig.m`。如果任一判据失败，则不会生成候选标定文件。

## 判定解释

- `VALIDATED_CANDIDATE_NOT_ACTIVATED`：数据支持“主要是常量零偏变化”，但仍需人工检查后才能决定是否启用。
- `BIAS_ONLY_REJECTED` 且 Joint/LoadOnly 明显更好：残差更可能与质量、CoG、姿态相关误差或线缆预载有关，不应直接把约 11 N 写入零偏。
- A 回位失败：实验过程中存在漂移、接触或线缆状态变化；本次数据不适合更新标定。
- 采集或运动中断：程序使用 V3 `Pause()` 暂停TCP运动队列并保存已有检查点，不会自动发送 `Continue()` 或执行回位恢复运动。继续下一次实验前，应确认并清理可能残留的队列目标。

默认验收阈值是当前工程的保守工程初值，不是传感器厂商精度声明；可根据稳定静止数据修改，但应在采集前固定，不能看完留出验证结果后再调阈值。
