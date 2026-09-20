# 阶段 2：机器人适配器、FK/Jacobian 与时序验证

## 1. 本阶段已实现的内容

阶段 2 的代码分为三层：

1. `scopeguide.io.RobotAdapter`
   - 将 CR5 反馈统一转换为 SI 单位；
   - 关节角：`deg -> rad`；
   - 关节速度：`deg/s -> rad/s`；
   - 控制器位置：`mm -> m`；
   - TCP 线速度：`mm/s -> m/s`；
   - 保存反馈序号、主机单调时间、反馈 age 和无效字节计数；
   - `connectReadOnly()` 只连接并读取反馈，不调用机器人使能或运动接口；
   - dry-run 下调用 `sendServoTarget()` 会直接抛出错误，不能下发命令。
2. 与网络完全解耦的 CR5 数学模型
   - `cr5ForwardKinematics.m`：标准 DH 正运动学；
   - `cr5GeometricJacobian.m`：法兰原点及内窥镜尖端的 6×6 空间几何 Jacobian；
   - 四元数、RPY 候选约定和 SO(3) 姿态误差工具。
3. 三种验证入口
   - 自动化单元测试和 Jacobian 中心差分验证；
   - 使用现有力标定记录进行历史多位姿验证；
   - 连接当前 CR5 后进行实机只读、多静止位姿及反馈时序验证。

本阶段不会调用 `Enable`、`ServoJ`、`MovJ`、`MovL` 或点动接口。实机脚本会建立
Dashboard、Move 和 Feedback TCP 连接，这是现有 `ZJFDobotCR5.Connect()` 的行为，但不会向
Dashboard 或 Move 连接写入任何命令。

## 2. 坐标和运动学约定

输入关节量统一为弧度。标准 DH 单节变换为：

$$
{}^{i-1}T_i = R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i),
\qquad \theta_i=q_i+\theta_{0,i}.
$$

法兰到内窥镜参考坐标系的已知变换为：

$$
{}^F T_E =
\begin{bmatrix}
1&0&0&-0.002840\\
0&1&0& 0.113051\\
0&0&1& 0.355046\\
0&0&0&1
\end{bmatrix}.
$$

因为内窥镜尖端就是坐标系 $E$ 的原点：

$$
{}^B T_E = {}^B T_F {}^F T_E.
$$

几何 Jacobian 在基坐标系表达。对于第 $i$ 个转动关节和控制点 $p$：

$$
J_{v,i}=z_{i-1}\times(p-o_{i-1}),\qquad
J_{\omega,i}=z_{i-1}.
$$

阶段 2 结题证据已确认 `CartesianPose` 是法兰位姿。验证程序仍分别计算：

- 控制器位姿 = 法兰位姿；
- 控制器位姿 = 内窥镜参考坐标系位姿。

哪一种位置误差显著更小，哪一种更符合当前控制器配置。三姿态实机只读记录及最终装配
12 姿态离线重放都支持“法兰”假设。

姿态计算以反馈四元数 `[qw,qx,qy,qz]` 为主。控制器 RPY 按
$R=R_z(r_z)R_y(r_y)R_x(r_x)$ 解释；两组结题证据中 RPY 与四元数的最大差分别为
$8.73\times10^{-7}$ rad 和 $1.27\times10^{-6}$ rad，因此该约定已写入阶段 2 结论。

## 3. 新增或修改的文件

| 文件 | 用途 |
|---|---|
| `+scopeguide/+io/RobotAdapter.m` | CR5 一致快照、SI 单位转换和命令拒绝 |
| `+scopeguide/+types/robotState.m` | invalid-by-default 的机器人状态结构 |
| `+scopeguide/+geometry/cr5ForwardKinematics.m` | 标准 DH FK、法兰和内窥镜尖端位姿 |
| `+scopeguide/+geometry/cr5GeometricJacobian.m` | 法兰/尖端空间几何 Jacobian |
| `+scopeguide/+geometry/rotationMatrixFromQuaternionWxyz.m` | 四元数转旋转矩阵 |
| `+scopeguide/+geometry/dobotRpyToRotation.m` | Dobot RPY 候选解释 |
| `+scopeguide/+geometry/rotationLogVector.m` | SO(3) 对数向量 |
| `+scopeguide/+geometry/rotationDistance.m` | 姿态距离 |
| `run_stage02_recorded_kinematics_validation.m` | 历史记录离线验证 |
| `run_stage02_robot_readonly_validation.m` | 当前 CR5 只读采集，不运动 |
| `aggregate_stage02_robot_readonly_validations.m` | 合并至少三个实机静止位姿 |
| `run_stage02_final_review.m` | 完全离线核对全部退出条件并生成结题报告 |
| `tests/testStage02RobotKinematics.m` | FK、Jacobian、单位和 dry-run 测试 |
| `tests/MockDobotBackend.m` | 不开网络、不运动的测试后端 |

此外，`ZJFDobotCR5.GetStateSnapshot()` 增加了当前反馈 age 和无效反馈字节计数。

## 4. 建议运行顺序

### 4.1 自动测试

从全新的 Matlab 会话运行：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');
clear functions;
results = runAllTests();
disp(table(results));
assert(all([results.Passed]));
```

阶段 2 的测试包括：

- 零位 FK 回归值；
- 随机姿态齐次变换合法性；
- 20 组随机姿态、六个关节的中心差分 Jacobian；
- 法兰到尖端的 Jacobian 平移恒等式；
- 把度误传给弧度接口时拒绝计算；
- RobotAdapter 单位转换、stale 反馈处理；
- dry-run 命令拒绝，且 mock 后端 `ServoJ` 调用次数保持为零；
- 历史多位姿数据的控制器位姿语义判断。

### 4.2 历史数据验证

```matlab
[historical, historicalDir] = ...
    run_stage02_recorded_kinematics_validation();
disp(historical.Summary);
disp(historicalDir);
```

这一步完全离线，读取现有力觉标定时保存的关节角、`CartesianPose`、四元数、反馈序号和
时间戳。默认读取 2026-08-12 最终装配数据，只保留当前标定采用的
2–6、8、9、11–15 号姿态、`stationary=true` 的行，并对重复机器人反馈帧去重。输出位于：

```text
results/stage02_recorded_kinematics_<timestamp>/
```

历史结果能用于发现明显的 DH、单位或位姿语义错误，但不能替代当前装配状态下的实机复核。

### 4.3 当前 CR5 的只读验证

运行前条件：

- 急停和机械限位状态正常；
- 机器人保持未使能或处于不会自行运动的状态；
- 机械臂和内窥镜由可靠支撑保护；
- PC 到 CR5 的 IP/端口与配置一致；
- 不运行其他会写机器人命令的 Matlab 脚本。

手动将机器人置于三个明显不同且安全的静止姿态。每次摆好并保持静止后，分别运行：

```matlab
[pose1, dir1] = run_stage02_robot_readonly_validation( ...
    PoseLabel="pose_01", DurationSec=10);
[pose2, dir2] = run_stage02_robot_readonly_validation( ...
    PoseLabel="pose_02", DurationSec=10);
[pose3, dir3] = run_stage02_robot_readonly_validation( ...
    PoseLabel="pose_03", DurationSec=10);
```

脚本不会替你移动机器人。三个姿态之间的移动必须由你按实验室已有的安全操作流程完成，
并且不能由本阶段代码完成。

合并三次结果：

```matlab
[stage02Summary, aggregateDir] = ...
    aggregate_stage02_robot_readonly_validations([dir1; dir2; dir3]);
disp(stage02Summary);
disp(aggregateDir);
```

### 4.4 完全离线结题审计

已经有三姿态汇总和最终装配离线报告后，可以直接运行：

```matlab
[review, reviewDir] = run_stage02_final_review();
disp(review);
disp(reviewDir);
assert(review.AllExitChecksPassed);
```

该入口会运行完整离线测试并读取已有证据，不连接机器人，也不重新采集数据。审计把
0.25 rad 作为“明显不同姿态”的最小关节向量距离；本次实测为 0.833574 rad。

## 5. 报告判读

### 5.1 阶段 2 最终复核（2026-08-13）

- 完整测试：68/68 通过，失败 0，incomplete 0；
- 三姿态实机只读：有效读取率均为 100%，stale=0，异常反馈字节增量=0，反馈序号回退=0；
- 三姿态之间最小关节向量距离：0.833574 rad；
- 适配器发送命令总数：0，物理运动命令：false；
- 实机反馈周期 P99 最大值：8.802 ms，反馈 age P99 最大值：8.051 ms；
- 数据建议 stale 阈值：26.407 ms，默认配置采用 30 ms；
- 三姿态实机最大法兰位置误差：0.964 mm；
- 最终装配 12 姿态、8700 个唯一反馈帧，最大法兰位置误差：0.983 mm；
- 最终装配 RPY/四元数最大差：$1.27\times10^{-6}$ rad；
- `CartesianPose` 参考对象：法兰；平移/转角原始单位：mm/deg；四元数顺序：wxyz；
- 结论：名义模型不支持 0.5 mm 软边界，但支持将 1.5 mm 硬边界作为后续验证候选。

正式结题报告保存在：

```text
results/stage02_final_review_20260813_164855_607/
```

最终装配离线报告保存在：

```text
results/stage02_recorded_kinematics_20260813_164236_368/
```

`cfg.robot.ControllerPoseReference`、`ControllerPoseUnitsVerified` 和 `ModelVerified` 已据此更新；
`ServoTimingVerified` 仍为 `false`，工具几何与 RCM 标定也仍未通过。

### 5.2 早期自动与历史验证（2026-08-07）

- 完整测试：37/37 通过；
- 阶段 2 专项测试：9/9 通过；
- 阶段 2 相关文件 Matlab Code Analyzer：零告警；
- 历史验证：1–9 号保留姿态、7107 行静止样本、4363 个唯一机器人反馈帧；
- 历史数据推断 `CartesianPose` 参考对象：法兰；
- 法兰位置 RMSE：0.8186 mm；最大值：1.0605 mm；
- 法兰姿态 RMSE：0.001649 rad（约 0.0945°）；最大值：0.002254 rad（约 0.129°）；
- RPY 候选解释与四元数最大差：约 $1.03\times10^{-6}$ rad；
- 反馈周期 P50/P95/P99：8.401/8.709/8.947 ms；
- 历史最大法兰位置误差是 0.5 mm 候选软边界的 2.121 倍，是 1.5 mm 候选硬边界的
  0.707 倍。

最新历史报告保存在：

```text
results/stage02_recorded_kinematics_20260807_021458_228/
```

这里的“支持法兰语义”很明确：若错误地把控制器位置当作内窥镜尖端，位置 RMSE 约为
371 mm，而法兰假设下约为 0.819 mm。不过，1.061 mm 最大误差距离 1.5 mm 候选硬边界
余量不大，因此该早期记录当时不能单独把 `ModelVerified` 改为 `true`；最终决策以后续三姿态
实机证据和 2026-08-12 最终装配 12 姿态重放为准。

### 5.3 实机报告字段

优先检查以下字段：

| 字段 | 含义 |
|---|---|
| `AdapterCommandSentCount` | 必须为 0 |
| `PhysicalMotionCommandSent` | 必须为 `false` |
| `ValidReadRatio` | 有效反馈读取比例，理想值为 1 |
| `StaleReadRatio` | 超过当前 stale 阈值的比例，理想值为 0 |
| `InvalidFeedbackByteCountIncrease` | 采集期间解析器丢弃的异常字节数 |
| `FeedbackPeriodP99Sec` | 30004 反馈的 P99 单帧周期 |
| `FeedbackAgeP99Sec` | 控制循环读到反馈时的 P99 age |
| `SuggestedFeedbackStaleSec` | 由实测时序给出的保守候选值，不自动写回配置 |
| `RpyQuaternionMismatchMaximumRad` | RPY 候选约定与四元数的最大姿态差 |
| `InferredControllerPoseReference` | 多位姿数据更支持法兰还是内窥镜尖端 |
| `FlangePositionMaximumM` | 名义 DH 法兰位置最大误差 |
| `FlangeErrorToHardRcmRatio` | 模型误差与候选硬 RCM 半径的比值 |

`SuggestedFeedbackStaleSec` 只是候选参数，检查网络负载下的重复实验后才能人工写回默认配置。

当前候选硬 RCM 半径是 1.5 mm。如果多位姿 `FlangePositionMaximumM` 与 1.5 mm 同量级或
更大，不能进入硬 RCM 实机阶段。需要先排查：

1. 控制器 `CartesianPose` 的参考对象；
2. DH 约定、关节零偏和关节方向；
3. 基坐标系/用户坐标系是否一致；
4. 控制器是否配置了活动 Tool/User 坐标系；
5. CR5 名义几何误差是否需要进一步标定。

## 6. 阶段 2 的退出边界

阶段 2 已于 2026-08-13 通过。结题审计确认：

- [x] 完整测试全部通过；
- [x] 至少三个实机静止位姿报告生成；
- [x] 三次报告均确认没有命令发送；
- [x] `CartesianPose` 参考对象、位置单位、四元数顺序和 RPY 约定得到证据支持；
- [x] P99 反馈周期、反馈 age、重复帧和异常字节有报告；
- [x] 已根据误差与 0.5/1.5 mm 候选 RCM 边界的比值完成人工决策。

`ServoJ` 命令到反馈延迟需要实际发送低速命令，超出本阶段“零运动命令”的授权范围，因此
这里只将 `ServoTimingMeasured` 保持为 `false`。它应在机械夹具、运动许可和前置阶段均通过后
单独验证，不能因为反馈周期已测得就假定命令延迟相同。
