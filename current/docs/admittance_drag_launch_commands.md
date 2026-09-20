# ScopeGuide 拖拽程序启动命令

本文记录当前两个可用拖拽版本的 MATLAB 启动命令：

1. 基于 RCM 的双轴 pivot + insertion 三自由度拖拽；
2. 不带 RCM 约束的 Cartesian 六自由度拖拽。

运行物理运动程序前，确认机械臂处于 TCP/IP 模式、HEX-H 已连接、
工作空间无障碍，并保持示教器和急停可触及。

## 通用按键

- `Space`：将软件运动使能锁存为 ON；
- `Esc`：立即将软件运动使能置为 OFF；
- `R`：在外力释放并满足复位条件后请求复位；
- 关闭图窗：请求停止程序，程序正常清理后会下使能机械臂；
- `Ctrl+C`：仅在正常停止机制失效时使用。

软件按键不是物理 deadman。程序会自动调用 `EnableRobot`、
`SpeedFactor`，并在正常退出和清理阶段调用 `DisableRobot`。

## 1. RCM 三自由度拖拽

控制自由度为：

```text
pivot1 + pivot2 + insertion
```

参数入口：`current_stage08_full_3dof_config.m`

启动命令：

```matlab
pool = gcp('nocreate');
if ~isempty(pool)
    delete(pool);
end

clear functions;
cd('/path/to/scopeguide-rcm-admittance-control/current');

% 加载当前RCM三自由度拖拽参数
[cfgFull3Dof, profileRcm] = ...
    current_stage08_full_3dof_config();

% 当前实际停止阈值：60/40 N、16/8 N*m。
% 参数文件中的历史确认短语仍需在启动时同步覆盖。
cfgFull3Dof.stage08.SafetyThresholdConfirmationPhrase = ...
    "CONFIRM_LIMITS_60N_40N_16NM_8NM";

[rcmSummary, rcmResultDir] = ...
    run_stage08_fixture_commissioning( ...
        Config=cfgFull3Dof, ...
        DurationSec=120, ...
        WindowSec=20, ...
        PlotRateHz=10, ...
        DofMode="full_3dof", ...
        ArmPhysicalMotion=true, ...
        MotionConfirmation="ENABLE_SCOPEGUIDE_MOTION", ...
        SetupConfirmation="STAGE8_FIXTURE_READY", ...
        SafetyThresholdConfirmation= ...
            "CONFIRM_LIMITS_60N_40N_16NM_8NM", ...
        SecondObserverPresent=true, ...
        FixtureAndClearanceConfirmed=true);
```

运行结果变量：

- `rcmSummary`：运行摘要；
- `rcmResultDir`：本次实验结果目录。

## 2. 无 RCM 六自由度拖拽

控制自由度为：

```text
X + Y + Z + Rx + Ry + Rz
```

参数入口：`current_cartesian_admittance_drag_config.m`

当前配置中平移和旋转的相对行程限制均已关闭：

```matlab
profile.TravelLimitsEnabled = false;
```

启动命令：

```matlab
pool = gcp('nocreate');
if ~isempty(pool)
    delete(pool);
end

clear functions;
cd('/path/to/scopeguide-rcm-admittance-control/current');

% 加载当前无RCM六自由度拖拽参数
[cfg6Dof, profile6Dof] = ...
    current_cartesian_admittance_drag_config();

% 确认相对位置和姿态行程限制已经关闭
assert(~cfg6Dof.cartesianDrag.TravelLimitsEnabled, ...
    '当前配置仍启用了Cartesian相对行程限制。');

[drag6DofSummary, drag6DofResultDir] = ...
    run_cartesian_admittance_drag( ...
        Config=cfg6Dof, ...
        DurationSec=120, ...
        WindowSec=20, ...
        PlotRateHz=10, ...
        DofMode="full_6dof", ...
        ArmPhysicalMotion=true, ...
        MotionConfirmation="ENABLE_SCOPEGUIDE_MOTION", ...
        SetupConfirmation="CARTESIAN_DRAG_FIXTURE_READY", ...
        SafetyThresholdConfirmation= ...
            "CONFIRM_LIMITS_60N_40N_20NM_10NM", ...
        SecondObserverPresent=true, ...
        FixtureAndClearanceConfirmed=true);
```

运行结果变量：

- `drag6DofSummary`：运行摘要；
- `drag6DofResultDir`：本次实验结果目录。

即使相对 Cartesian 行程限制已关闭，以下保护仍然有效：

- 机器人关节位置限制；
- 关节速度和加速度限制；
- 奇异位形降速/停止；
- ServoJ 单周期目标步长限制；
- 目标—反馈跟踪误差保护；
- HEX-H 原始/快速力和力矩安全停止。

## 修改运行时间

两个程序均可修改：

```matlab
DurationSec=120
```

例如运行 10 分钟：

```matlab
DurationSec=600
```

`WindowSec=20` 只控制实时曲线显示窗口长度，不限制结果文件保存时长。
