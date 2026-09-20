# ScopeGuide 当前控制程序

本目录包含截至 2026-08-30 的工作代码快照：RCM 三自由度拖拽、无 RCM 的 Cartesian 六自由度拖拽，以及共享的通信、标定、控制和离线测试模块。

| 模式 | 参数入口 | 运行入口 |
|---|---|---|
| RCM：pivot1、pivot2、insertion | `current_stage08_full_3dof_config.m` | `run_stage08_fixture_commissioning.m` |
| Cartesian：X、Y、Z、Rx、Ry、Rz | `current_cartesian_admittance_drag_config.m` | `run_cartesian_admittance_drag.m` |

完整的实机启动命令见 [启动说明](docs/admittance_drag_launch_commands.md)。先进入本目录；不要对整个仓库使用 `addpath(genpath(...))`，当前版、根目录初始版和归档版包含同名 MATLAB 包，必须在不同的 MATLAB 会话中运行。

## 依赖与资源

- MATLAB R2025b；Optimization Toolbox（`quadprog`）；Parallel Computing Toolbox（process worker）。
- Dobot CR5 TCP/IP 接口与 OnRobot HEX-H；HEX-H 可使用 `tcpclient` 后端，`modbus` 后端依赖相应工具箱。
- `resources/validated_base_config.mat`：原 dual-pivot 实验保存的基础配置；`validated_pivot2_config.mat`：早期 dual-pivot 配置入口所需资源。
- `force_calibration/data/`：负载标定 JSON；附带两份历史多姿态 CSV，用于已有的回放回归测试。
- `force_sensor/data/`：两份历史力传感器 CSV，供离线回放测试使用。

配置入口默认加载随代码打包的 MAT 文件，并将运行路径解析到本目录。`SourceAcceptedConfigFile` 显式指定的外部配置保留自己的路径及标定；使用外部配置时需要自行检查这些资源是否可用。MAT 文件内的历史路径属于原实验元数据，不是默认运行时依赖。

硬件地址、工具变换、RCM 坐标、标定和验收记录对应原实验安装，不是新设备的通用标定。控制参数、运动确认门和力/力矩保护保持工作版本的设置。当前六自由度参数关闭了相对 Cartesian 行程限制，具体含义见启动说明。

## 离线验证

在本目录启动 MATLAB：

```matlab
results = runAllTests();
disp(table(results));
```

测试使用离线模型、记录数据和模拟通信后端，不调用实机启动命令。发布整理的具体检查结果见 [验证记录](docs/publication_validation.md)。

## 模块

- `+scopeguide/+control/`：RCM/Cartesian 导纳、QP、限速与关节积分。
- `+scopeguide/+geometry/`：CR5 运动学、RCM 几何、拖拽点运动学。
- `+scopeguide/+force/`：补偿、基线、滤波、死区与力安全监测。
- `+scopeguide/+runtime/`、`+scopeguide/+safety/`、`+scopeguide/+ui/`：控制 worker、运动授权和显示。
- `robot/`、`force_sensor/`、`force_calibration/`：硬件通信与标定。
- `tests/`、`run_*validation.m`、其他诊断入口：离线回归及分阶段诊断。

运行结果写入本目录下的 `results/`，不纳入版本管理。未打包完整实验结果和全部历史采集数据；需要额外历史数据的诊断脚本应显式提供对应输入文件。
