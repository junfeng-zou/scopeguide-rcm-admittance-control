# HEX-H 力觉补偿标定

本目录从 `vision-force-safety` 提取，只保留力觉负载标定程序和已经采集的
标定数据。完整 HEX-H 读取、滤波、记录和处理模块位于项目根目录的 `force_sensor/`。

## 内容

- `calibrateHexHToolLoadWithDobot.m`：采集多姿态静止六维力并完成标定。
- `fitHexHToolLoadCalibration.m`：根据已采集样本拟合工具质量、质心和六维零偏。
- `compensateHexHWrench.m`：使用标定结果进行零偏和姿态相关重力补偿。
- `loadHexHGravityCalibration.m`：加载 JSON 标定结果。
- `createExcludedPoseCalibration.m`：从已保存的完整批次中排除异常姿态并重新拟合。
- `../force_sensor/+onrobot/`：在线标定和后续控制共用的 HEX-H Modbus TCP 客户端与处理模块。
- `data/hex_h_tool_calibration_20260807_164630/`：当前完整标定批次及排除 Pose 9 的正式候选。
- `data/hex_h_tool_calibration_20260722_182635/`：历史标定批次，保留用于追溯。
- `data/validation/`：坐标系和真实标定验证结果。

## 当前采用的标定结果

当前采用的是 2026-08-07 重新采集的数据中保留姿态 1–8、10，排除异常
Pose 9 的结果：

```matlab
calibrationFile = fullfile(fileparts(mfilename('fullpath')), 'data', ...
    'hex_h_tool_calibration_20260807_164630', ...
    'calibration_without_pose09.json');
```

主要参数：

- 工具质量：`1.2223682085391223 kg`
- 质心（相对 HEX-H 测量原点，传感器坐标系）：
  `[-0.0040120767, -0.0615630725, 0.0520286763] m`
- 传感器六维零偏：
  `[-0.537776476, 2.029313569, -4.084993597, 0.246517723, 0.195914386, -0.113815668]`
- 传感器到工具坐标旋转：`diag([-1, -1, 1])`
- 重力模型符号：`+1`
- 最大设计矩阵条件数：`13.9461`
- 九姿态留一力/力矩 RMSE：`0.2621 N / 0.0570 N·m`
- 被排除 Pose 9 对九姿态模型的残差：约 `2.55 N / 0.41 N·m`
- 7200 点保留姿态回放：全部有效、零安全告警/停止，最大慢速残余合力
  `1.177 N`，经过当前 `1.3 N` 死区后最大控制力为 `0 N`

## 使用

在项目根目录执行：

```matlab
addpath(fullfile(pwd, 'force_sensor'));
addpath(fullfile(pwd, 'force_calibration'));

calibrationFile = fullfile(pwd, 'force_calibration', 'data', ...
    'hex_h_tool_calibration_20260807_164630', ...
    'calibration_without_pose09.json');
[calibration, provenance] = loadHexHGravityCalibration(calibrationFile);

% wrenchSensor: [Fx; Fy; Fz; Tx; Ty; Tz]
% quaternionWxyz: Dobot base_from_tool 四元数 [qw,qx,qy,qz]
result = compensateHexHWrench(wrenchSensor, quaternionWxyz, calibration);
```

## 外部依赖

重新执行在线标定时，`calibrateHexHToolLoadWithDobot.m` 还需要项目中的
`force_sensor/`、`ZJFDobotCR5` 和 `decodeDobotFeedbackFrames`。两个机器人驱动文件
不属于力觉标定模块，因此没有复制进本目录。已有数据的离线拟合、加载和补偿只需
`force_calibration/`。
