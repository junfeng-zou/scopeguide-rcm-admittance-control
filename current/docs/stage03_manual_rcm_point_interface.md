# 阶段 3 暂缓：人工 RCM 基座坐标接口

## 当前决定

阶段 3 的多轴线 RCM 实测标定暂缓。后续直接提供 RCM 点在机器人基座坐标系
`robot_base` 下的三维坐标。该决定允许项目先进入阶段 4 的离线状态机工作，但不表示
阶段 3 已通过。

以下项目仍未验证：

- 最终工具矩阵、尖端位置和镜杆轴向；
- RCM 多次标定重复性；
- 软、硬 RCM 边界；
- 夹具或头模移动后的重新标定流程。

2026-08-16，用户明确确认当前工具矩阵、RCM 坐标和 `0.5/2.0 mm` 软硬边界可直接用于
当前不变装配下的自由空间 commissioning。因此默认配置现将 `GeometryVerified`、
`CalibrationValid` 和 `BoundsValidated` 设为 `true`。这是一项用户实物确认，不等价于
多轴线计量报告；装配、传感器、内窥镜、夹具或 RCM 位置变化后确认立即失效。版本化记录：
`config/stage08_user_attestation_20260814.json`。

## 坐标注入方法

如果提供的是毫米：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');
addpath('config');
cfg = defaultRcmAdmittanceConfig();

[cfg, rcmRecord] = applyManualRcmPoint(cfg, ...
    [x_mm; y_mm; z_mm], ...
    InputUnit="mm", ...
    SourceDescription="RCM point supplied in robot base frame", ...
    FixtureIdentifier="填写当前夹具或头模编号");
```

如果提供的是米，把 `InputUnit` 改成 `"m"`。函数输出：

- `cfg.rcm.PointBaseM`：统一为米的 3×1 坐标；
- `cfg.rcm.PointFrame="robot_base"`；
- `cfg.rcm.PointSource="manual_base_coordinate"`；
- `rcmRecord`：输入单位、毫米/米坐标、来源、夹具编号和记录时间。

## 安全行为

单独调用 `applyManualRcmPoint` 输入一个新坐标时仍然保持：

```matlab
cfg.rcm.CalibrationValid = false;
cfg.rcm.BoundsValidated = false;
```

因此新输入操作本身只允许后续离线几何计算和 dry-run 接线，不会继承旧坐标的用户确认。等坐标、
来源、当前夹具状态和边界都确认后，再建立版本化 RCM 记录并单独审核有效性。

夹具、头模、机器人基座或入口位置一旦移动，人工输入的旧坐标必须作废。
