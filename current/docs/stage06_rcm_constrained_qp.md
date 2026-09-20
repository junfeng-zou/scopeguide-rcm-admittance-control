# 阶段 6：软/硬/混合 RCM 约束 QP

## 完成状态

阶段 6 已于 2026-08-13 通过纯离线结题审计：

- `quadprog` 路径和 Optimization Toolbox 许可证已实际验证；
- 全量离线回归：102/102；
- 阶段 6 专项测试：11/11；
- 300 姿态合成工作区扫描：296/300 成功，98.67%；
- 4 个拒绝全部是预期的 `JACOBIAN_SINGULAR_STOP`，不可解释失败为 0；
- 预热后 `quadprog` P50/P95/P99：0.579/0.698/0.810 ms；
- 成功样本最大约束违反：`2.38e-14`；
- Matlab Code Analyzer：阶段 6 新增/修改文件零告警；
- 硬件连接：0；机器人运动命令：0。

正式报告：

```text
results/stage06_final_review_20260813_231044_844/
```

## 实现组成

| 文件 | 用途 |
|---|---|
| `+scopeguide/+geometry/computeRcmConstraintKinematics.m` | RCM 横向误差、最近轴线点和解析约束 Jacobian |
| `+scopeguide/+control/solveRcmConstrainedQp.m` | 无状态 `quadprog` 问题构造、求解与 fail-closed 检查 |
| `+scopeguide/+control/RcmConstrainedQpController.m` | 上一周期速度、实际相对坐标、连续失败与阶段 4 复位合同 |
| `+scopeguide/+control/warmupRcmQpSolver.m` | PREARM 前显式求解器预热 |
| `run_stage06_qp_workspace_scan.m` | 三种模式的合成工作区可行性和时延扫描 |
| `run_stage06_final_review.m` | 阶段 6 正式离线结题审计 |
| `tests/testStage06RcmConstrainedQp.m` | Jacobian、约束、模式、失败、奇异和工作区测试 |

## RCM 误差与约束

令 `pTip` 为尖端、`a` 为镜杆轴、`pRcm` 为 RCM 点：

```text
s        = dot(a, pRcm - pTip)
pClosest = pTip + s*a
eRcm     = pRcm - pClosest
e2       = [b1 b2]' * eRcm
```

`C(q)` 表示镜杆轴线上最近点的两个横向速度：

```text
C = [b1 b2]' * [I, -skew(s*a)] * Jtip
```

因此 `C*qdot = kR*e2` 会让轴线点朝当前 RCM 误差方向运动，而误差更新为：

```text
e2_next = e2 - C*qdot*dt
```

这里只约束两个横向自由度，不会错误禁止沿镜杆轴向插入。

## 三种模式

- `hard`：将 `C*qdot=kR*e2` 作为等式约束；
- `soft`：使用固定权重的 RCM 恢复目标；
- `hybrid`：RCM 误差从 soft 半径接近 hard 半径时，连续增加恢复权重和增益；
- 三种模式都保留预测 hard 包络，当前误差已经超过 hard 半径时直接输出零；
- 当前默认仍是 `hybrid`。

soft 和 hybrid 不是无边界伪逆：硬 RCM 包络始终作为安全不等式存在。

## QP 目标与单位缩放

QP 目标包括：

1. 3-DOF 导纳产生的尖端 twist 跟踪；
2. soft/hybrid RCM 横向恢复；
3. 小关节速度正则；
4. 朝对称关节范围中点的低权重速度偏好。

线速度保持 `m/s`，角速度乘 `0.15 m` 特征长度后进入同一个最小二乘目标，避免米与弧度
直接相加。

## 强制约束

- 每关节速度上下限；
- `qdotPrevious ± qddotMaximum*dt`；
- 一步预测关节位置和离线关节范围；
- 当前和预测 RCM hard 包络；
- 相对使能起点的二维 pivot 范数与 insertion 边界；
- 约束由 32 边内接多边形保守近似圆形边界；
- 求解后再次检查真实圆形范数和全部数值残差，违反即丢弃结果并输出零。

QP 结果不会再进行逐关节剪切。奇异性处理对整个期望 twist 和恢复目标使用同一个连续比例；
低于停止奇异值时输出严格的六轴零速度。

## 失败策略

以下情况当周期 `QdotCommandRadSec=zeros(6,1)`：

- `quadprog` 不可用；
- 当前 RCM 或相对行程已经越界；
- 关节速度/加速度/一步位置边界互相矛盾；
- `exitflag<=0`、不可行或求解异常；
- 求解时间超过 5 ms；
- 约束后验检查不通过；
- Jacobian 进入停止奇异区。

`RcmConstrainedQpController` 在前三次失败期间仍拒绝当周期命令；连续三次失败后输出
`RequiresFault=true`，供阶段 4 锁存 FAULT。松手或 `ResetDynamicState=true` 会同步清除上一
周期速度和实际 pivot/insertion 相对坐标。

## quadprog 预热

新 Matlab 会话中的第一次 `quadprog` 调用存在 JIT/缓存开销，实测可达几十毫秒。所有时延
统计均在显式预热后进行；未来阶段 7 的运行入口必须在进入 PREARM 前调用：

```matlab
cfg = defaultRcmAdmittanceConfig();
warmup = scopeguide.control.warmupRcmQpSolver(cfg);
assert(warmup.ReadyForTimingAudit);
```

当前报告中的 P99 仅覆盖预热后的 `quadprog` 调用。完整力处理、FK、导纳、QP 和日志控制周期
时延必须在阶段 7 实机只读 dry-run 中重新测量。

## 运行方法

工作区扫描：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');
[scan, scanDir] = run_stage06_qp_workspace_scan( ...
    SampleCount=300);
```

正式复核：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');
[review, reviewDir] = run_stage06_final_review();
```

## 安全边界

阶段 6 使用的是“每个代表性关节姿态对应一个合成轴线 RCM 点”，只验证 QP 数学、约束和
求解时间，不声称真实 RCM 已标定。当前依然保持：

- `EnableMotion=false`、`DryRun=true`；
- 阶段 3 未完成；
- `CalibrationValid=false`、`BoundsValidated=false`；
- 关节范围和 RCM 半径仍未经过实机验证；
- 软件键鼠不能授权物理运动。

进入阶段 7 前，需要用户提供真实基座坐标 `pRcmBase`，并单独准备机器人和 HEX-H 的只读
连接实验；阶段 7 仍不会调用 ServoJ。
