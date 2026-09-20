# 阶段 5：3-DOF RCM 导纳（六维 wrench 更新）

## 完成范围

阶段 5 完成的是纯数学与已有静态数据重放，不连接机器人或力传感器，也不发送运动命令。
由于阶段 3 的真实 RCM 标定按当前决定暂缓，自动验收使用明确标记的合成
`pRcmBase=[0,0,0] m` 和 150 mm RCM—尖端距离。真实 `pRcmBase` 仍通过
`config/applyManualRcmPoint.m` 注入；没有真实坐标不影响本阶段算法验收，但继续阻断真实运动。

阶段 5 已于 2026-08-13 通过离线结题审计：

- 全量离线回归：91/91；
- 阶段 5 专项测试：10/10；
- Matlab Code Analyzer：阶段 5 新增文件零告警；
- 当前静态记录：启动基线完成后的 600 个 eligible 样本无持续导纳漂移；
- 硬件连接：0；机器人运动命令：0。

正式报告：

```text
results/stage05_final_review_20260813_225054_846/
```

## 实现组成

| 文件 | 用途 |
|---|---|
| `+scopeguide/+geometry/computeRcmMotionBasis.m` | 稳定构造 `b1/b2/a` 与 3-DOF RCM twist 基 |
| `+scopeguide/+control/mapWrenchToRcmGeneralizedEffort.m` | 将 HEX-H 原点六维 wrench 平移到 RCM，再映射为 pivot/pivot/insertion 广义输入 |
| `+scopeguide/+control/deriveForceOnlyAdmittanceParameters.m` | 由设计力、目标速度、时间常数推导 `D/M` |
| `+scopeguide/+control/ForceOnlyRcmAdmittance.m` | 质量—阻尼导纳、soft-start、限幅、状态同步和复位 |
| `run_stage05_force_only_admittance_replay.m` | 对现有静态标定记录做离线导纳重放 |
| `run_stage05_final_review.m` | 阶段 5 正式结题审计 |
| `tests/testStage05ForceOnlyRcmAdmittance.m` | 方向、性质、限幅、停止与数据重放测试 |

## 数学约定

广义速度状态顺序为：

```text
u = [pivot1_rad_s; pivot2_rad_s; insertion_m_s]
```

给定在 HEX-H 测量原点、以工具轴表达的 `F_sensor, M_sensor`，先旋转到基座坐标并平移参考点：

```text
M_RCM = M_sensor + (p_sensor - p_RCM) × F
g1 = dot(b1, M_RCM)
g2 = dot(b2, M_RCM)
g3 = dot(Fbase, insertionDirection)
```

当前 `p_sensor` 使用法兰 `+Z 20 mm` 的用户估计。`MomentUsedForControl=true`；roll 未进入状态向量，输出恒为严格的零。
`b1/b2` 的参考轴在工具坐标中由固定镜杆轴一次确定，机器人姿态变化不会触发参考轴切换。

每轴导纳为：

```text
uDot = (g - D*u) / M
uNext = u + uDot*dt
```

其中：

```text
D = design generalized effort / target steady velocity
M = D * time constant
```

pivot 的设计广义输入为 `DesignForceN * NominalLeverArmM`，insertion 为
`DesignForceN`。代码没有显式矩阵求逆。

## 冻结首版参数

| 参数 | Pivot | Insertion |
|---|---:|---:|
| 设计力 | 5 N | 5 N |
| 名义力臂 | 150 mm | — |
| 目标稳态速度 | 0.5 deg/s | 1 mm/s |
| 时间常数 | 0.30 s | 0.30 s |
| 最大速度 | 1 deg/s | 2 mm/s |
| 最大加速度 | 5 deg/s² | 10 mm/s² |
| 相对使能起点边界 | 5 deg（二维范数） | 5 mm |

pivot 另受 5 mm/s 尖端线速度上限约束：RCM—尖端距离越长，允许的 pivot 角速度越低。

## 阶段 4 接口

- `MotionPermitted=false`：当周期输出严格为零，并清除速度和相对坐标；
- `ResetDynamicState=true`：立即清除导纳状态；
- `CommandScale`：只缩放外力输入，阻尼衰减保持有效，实现 200 ms soft-start；
- 非有限力、无效几何或 `dt > MaximumDtSec`：fail closed，返回零并请求故障；
- 所有限速和边界处理后的值写回内部状态，不保存未受限的隐藏状态。

## 运行方法

静态记录重放：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');
[replay, resultDir] = run_stage05_force_only_admittance_replay();
```

正式结题复核：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');
[review, resultDir] = run_stage05_final_review();
```

重放入口会打开已有记录所需的 AutomaticBaseline，并仅对这份“每个 capture 都是人工静置、
无接触”的标定 CSV 使用 `AssumeSettledStaticPose=true`。这是显式离线数据语义，不会改变实时
pipeline，也不会修改默认的实时 `RobotStationary` 门。

## 安全边界

阶段 5 通过不表示可以驱动机器人。当前仍有以下硬阻断：

- `cfg.robot.EnableMotion=false`、`DryRun=true`；
- 阶段 3 未完成；
- `cfg.rcm.CalibrationValid=false`；
- `cfg.rcm.BoundsValidated=false`；
- ServoJ 时序和实机 RCM/QP 尚未验证；
- 软件键盘/鼠标不能作为真实物理 deadman。

下一阶段可先在合成几何中完成阶段 6 QP；涉及真实机器人状态的 dry-run 前，必须注入用户提供
的基座系 `pRcmBase`。
