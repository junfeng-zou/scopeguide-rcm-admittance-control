# 无 RCM 约束的笛卡尔导纳拖拽

## 实现概览

这条控制链与原有 RCM 三自由度控制链相互独立。它复用已经验证的：

- HEX-H 采集、重力/零偏补偿、20 Hz Fast 与 5 Hz Slow 滤波；
- 软件启动置零、空闲 Fz 漂移跟踪与操作时冻结基线；
- 力/力矩死区和独立的原始/快速安全停止监视器；
- Space 锁存 ON、Esc 立即 OFF、R 故障复位；
- 20 Hz 后台控制 worker、非阻塞 ServoJ 和前台绘图；
- 机器人程序使能、关节限位、命令步长和跟踪误差保护。

它明确删除了以下内容：

- RCM 点坐标；
- RCM 横向误差；
- RCM 软/硬约束；
- RCM 恢复速度；
- QP 中的 RCM 等式或不等式行。

控制变量改为拖拽点的六维速度：

```text
[vx, vy, vz, wx, wy, wz]
```

默认拖拽点是 HEX-H 测量原点，即法兰 `+Z 20 mm`，坐标轴与法兰/工具控制轴对齐。测量力矩会先从 HEX-H 测量原点平移到配置的拖拽点，然后进入六维导纳。

## 控制流程

```text
HEX-H 六轴力/力矩
  -> 重力、零偏、滤波、基线和死区
  -> 力矩平移到拖拽点
  -> 6D 虚拟质量—阻尼导纳
  -> 相对位姿范围与制动距离限制
  -> 无 RCM 关节速度 QP
  -> 关节速度/加速度/位置限制
  -> 关节目标积分
  -> ServoJ
```

六维导纳采用：

```text
M * dv/dt + D * v = wrench
D = 设计力（或力矩） / 设计稳态速度
M = D * 时间常数
```

QP 跟踪拖拽点笛卡尔速度，同时保留关节速度、关节加速度、关节位置、奇异性和相对位姿边界。QP 不读取 `cfg.rcm`。

## 可用自由度模式

| `DofMode` | 开放自由度 |
|---|---|
| `hold` | 全部关闭，仅检查系统 |
| `translation_z` | 工具局部 Z 平移 |
| `translation_xyz` | 三轴平移 |
| `rotation_x` / `rotation_y` / `rotation_z` | 单轴旋转 |
| `rotation_xyz` | 三轴旋转 |
| `full_6dof` | 三轴平移和三轴旋转 |

建议首次实机运行从 `translation_z` 开始，确认方向、停止和范围后再逐项开放，最后使用 `full_6dof`。

## 参数文件

所有可调参数集中在：

```text
current_cartesian_admittance_drag_config.m
```

初始值为：

- 平移设计力：5 N；
- 平移设计稳态速度：3 mm/s；
- 平移最大速度：6 mm/s；
- 平移时间常数：0.20 s；
- 三轴相对平移范围：各 ±10 mm；
- 旋转设计力矩：0.50 N·m；
- 旋转设计稳态速度：2 deg/s；
- 旋转最大速度：4 deg/s；
- 旋转时间常数：0.20 s；
- 三轴相对旋转范围：各 ±5 deg；
- 控制用力/力矩饱和：15 N / 1.0 N·m；
- 关节速度上限：6 deg/s；
- 控制频率：20 Hz。

控制用饱和值不是安全停止阈值。安全监视仍独立使用 60 N 原始力、40 N Fast 力、4 N·m 原始力矩和 2 N·m Fast 力矩。

## 首次平移 Z 测试命令

```matlab
pool = gcp('nocreate');
if ~isempty(pool)
    delete(pool);
end

clear functions;
cd('/path/to/scopeguide-rcm-admittance-control/current');

[cfgFree, freeProfile] = ...
    current_cartesian_admittance_drag_config();

[freeSummary, freeDir] = run_cartesian_admittance_drag( ...
    Config=cfgFree, ...
    DurationSec=60, ...
    WindowSec=20, ...
    PlotRateHz=10, ...
    DofMode="translation_z", ...
    ArmPhysicalMotion=true, ...
    MotionConfirmation="ENABLE_SCOPEGUIDE_MOTION", ...
    SetupConfirmation="CARTESIAN_DRAG_FIXTURE_READY", ...
    SafetyThresholdConfirmation= ...
        "CONFIRM_LIMITS_60N_40N_4NM_2NM", ...
    SecondObserverPresent=true, ...
    FixtureAndClearanceConfirmed=true);
```

启动软件置零完成后，按 Space 锁存使能。Esc 立即关闭软件运动输入；发生 Fault 后先撤力、按 Esc，确认安全并等待力回到中性区，再按 R 复位。

## 六自由度启动

只有完成逐轴方向和停止检查后，才把上面命令中的模式改为：

```matlab
DofMode="full_6dof"
```

## 结果文件

每次运行写入：

```text
results/cartesian_drag_no_rcm_<模式>_<时间戳>/
```

其中包含：

- `cartesian_drag_control.csv`：力/力矩、目标和实际笛卡尔速度、相对位姿、关节速度及关节目标；
- `config_snapshot.json`：本次完整配置；
- `authorization.json`：物理运动门检查；
- `summary_worker.json` 与 `summary.json`：频率、ServoJ、跟踪、停止及故障摘要；
- `cartesian_drag_worker_result.mat`：MATLAB 汇总结构。

## 当前限制

- Space/Esc 是软件锁存输入，不是物理 deadman；示教器和急停必须始终可触及。
- 当前拖拽点采用法兰 `+Z 20 mm` 的测量原点估计。若以后安装正式握把，建议把 `TFlangeDragPoint` 改为握把中心，以便力矩平移和旋转手感与真实施力点一致。
- 相对旋转边界使用小角度线性预测，但实际相对姿态由 SO(3) 对数重新计算；当前正常范围为 ±10 deg。
- 正常边界外设有恢复区：平移 0.5 mm、旋转 0.3 deg。恢复区内禁止继续向外运动但允许向内返回；只有超出外层硬边界才触发 QP 失败。
- `full_6dof` 比原 RCM 三自由度更容易接近机器人奇异位形或关节边界，因此必须按单轴到全轴的顺序验收。
