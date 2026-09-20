# 法兰到内窥镜镜尖的固定点标定

## 目的

`run_flange_endoscope_tip_calibration.m` 通过十个固定镜尖位姿，估计内窥镜镜尖在机械臂法兰坐标系中的位置：

```text
p_tip_base = p_flange_base + R_base_flange * p_tip_flange
```

实验期间必须让内窥镜物理镜尖始终落在同一个刚性凹点中，只改变法兰姿态。程序只读取 Dobot 30004 反馈，不会调用 `EnableRobot`、`DisableRobot`、`ServoJ` 或其他运动命令。

## 重要限制

固定点实验只能标定法兰到镜尖的三维平移，不能从单个固定点辨识内窥镜坐标系的旋转。程序会保留输入配置中现有的 `cfg.tool.TFlangeEndoscope(1:3,1:3)`；当前默认值为单位阵。

因此，输出的齐次矩阵中：

- 平移：由本次十位姿数据重新估计；
- 旋转：从当前配置复制，并非本次实验测得。

## 实验准备

1. 将完整末端负载和内窥镜可靠安装好。
2. 准备一个不会移动、不会变形的刚性小凹点，用于固定镜尖位置。
3. 保持示教器和急停可触及。改变位姿时使用示教器/人工引导，松开后等待机械臂完全静止。
4. 镜尖放入凹点后，后续十个位姿都不能滑出或更换接触点。

## 启动命令

```matlab
clear functions;
cd('/path/to/scopeguide-rcm-admittance-control/current');

[tipReport, tipDir] = ...
    run_flange_endoscope_tip_calibration( ...
        PoseCount=10, ...
        SamplesPerPose=50);
```

程序提示后输入：

```text
CALIBRATE_FLANGE_TIP
```

每次将机械臂拖到新姿态、完全静止后，依次输入：

```text
SAMPLE 1
SAMPLE 2
...
SAMPLE 10
```

## 位姿如何选择

- 镜尖位置必须固定，但法兰方向应明显变化。
- 同时覆盖正负方向的俯仰、偏航和滚转，不要十次都只绕同一根轴转动。
- 相邻新姿态与已有姿态至少相差约 8°，全部姿态的方向跨度至少约 30°。
- 避开机械臂奇异位形、关节极限和内窥镜/线缆受拉状态。
- 每次输入 `SAMPLE n` 前先松手并等待机械臂、镜杆和线缆完全稳定。
- `RobotMode` 只写入原始记录用于追溯，不参与采样或计算的放行判断。

## 验收与结果

只有 `tipReport.Valid == true` 时才建议采用新平移。默认验收条件包括：

- 设计矩阵满秩且条件数不超过 `1e4`；
- 姿态总跨度至少 `30°`；
- 固定点残差 RMS 不超过 `1.0 mm`；
- 单个位姿最大残差不超过 `2.0 mm`。

结果保存在：

```text
results/flange_endoscope_tip_calibration_时间戳/
```

主要文件：

- `capture_checkpoint.mat`：每接受一个位姿后更新，可用于保留已采集数据；
- `raw_samples.csv`：每一帧原始机器人反馈；
- `pose_summary.csv`：十个位姿均值、稳定性和残差；
- `calibration.mat` / `calibration.json`：完整标定结果；
- `fixed_tip_residuals.png`：各位姿固定点残差图。

程序不会自动覆盖 `defaultRcmAdmittanceConfig.m`。先人工检查有效性、残差和输出矩阵，再决定是否更新当前工具参数。
