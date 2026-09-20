# 阶段 4：软件 deadman 状态机与命令积分器

## 完成状态

阶段 4 已于 2026-08-13 通过：

- 全量离线回归：81/81；
- 阶段 4 专项时间线与随机序列性质测试：10/10；
- Matlab Code Analyzer：阶段 4 新增/修改文件零告警；
- 硬件连接：0；
- 机器人运动命令：0。

正式报告：

```text
results/stage04_final_review_20260813_223424_224/
```

本次正式复验同时记录了用户人工操作确认：键盘自动重复修正后以及鼠标按住方式均已无明显
使用问题。人工确认保存在结题目录的 `manual_acceptance.md`；它只确认 GUI 可用性，自动安全
结论仍以 `81/81` 回归、`10/10` 专项测试和零硬件命令证据为准。

## 实现组成

| 文件 | 用途 |
|---|---|
| `+scopeguide/+io/EnableHandleAdapter.m` | 外部布尔量、时间戳、stale 和来源质量处理 |
| `+scopeguide/+control/HandleEnableStateMachine.m` | `DISABLED/PREARM/ENABLED/STOPPING/FAULT` 状态机 |
| `+scopeguide/+control/JointCommandIntegrator.m` | 实测 `dt`、soft-start、限速、限加速度和重锚 |
| `+scopeguide/+types/enableSafetyStatus.m` | neutral、力/机器人新鲜度、QP 和周期健康合同 |
| `run_stage04_software_deadman_demo.m` | 空格键/鼠标按住离线演示和日志 |
| `run_stage04_final_review.m` | 完全离线结题审计 |
| `tests/testStage04DeadmanAndIntegrator.m` | 必做模拟时间线测试 |

## 冻结行为

- 上升沿连续满足安全条件 40 ms 后才从 `PREARM` 进入 `ENABLED`；
- PREARM 检测到明显预载时保持 PREARM，不输出运动许可；
- `ENABLED` 使用 200 ms soft-start；
- 鼠标、外部布尔输入或已确认的键盘松手不做状态机下降沿防抖，当周期立即进入
  `STOPPING`，速度清零并重锚；
- Linux/X11 长按空格可能产生相隔约 10 ms 的伪 `KeyRelease/KeyPress`；键盘 GUI 层采用
  50 ms 释放确认窗过滤该事件对。真正空格松开因此最多增加约 50 ms 确认延迟；
- `Esc`、鼠标松开和窗口关闭绕过键盘确认窗，立即撤销软件使能；
- 输入时间戳超过 20 ms 未更新，在 PREARM/ENABLED 中进入锁存 FAULT；
- 力 stale、机器人反馈 stale、QP 失败或控制超周期均锁存 FAULT；
- 故障消失不会自动恢复，必须松开输入并按 `R` 显式 reset；
- 停止与故障路径清零积分器的上一周期速度，避免再次使能追赶旧目标；
- `qTarget-qState`、关节速度及关节加速度均有限幅；
- 所有软件输入均标记 `SupportsPhysicalMotion=false`。

`ResetDynamicState=true` 是阶段 4 留给阶段 5 的合同：以后导纳内部速度、滤波或速率限制
状态必须在该标志出现时同步清零。

## 键盘/鼠标离线演示

在 Matlab 中运行：

```matlab
cd('/path/to/scopeguide-rcm-admittance-control/current');
[summary, resultDir] = run_stage04_software_deadman_demo( ...
    DurationSec=60);
```

操作方法：

- 按住 `SPACE`：请求使能；
- 松开 `SPACE`：停止；
- 按住蓝色鼠标区域：请求使能；
- 松开鼠标：停止；
- `Esc`：撤销当前软件输入；
- `R`：输入已松开且健康条件恢复后请求 FAULT reset；
- 关闭窗口：强制结束并重锚。

用 `Ctrl+C` 中断时，`onCleanup` 会绕过窗口的普通关闭回调并强制删除 figure。键盘、鼠标
和关闭回调的状态保存在 figure `appdata` 中，不依赖已经被中断的函数工作区，所以窗口右上角
关闭按钮在异常中断后仍可工作。在 MATLAB 正处于 `drawnow` 图形刷新时，第一次 `Ctrl+C`
可能只中断刷新；如果命令提示符尚未返回，再按一次即可。

若需要清除旧版本遗留的卡死窗口，回到命令行运行：

```matlab
close_stage04_software_deadman_demo();
```

这个命令会按窗口名称或 Tag 查找，并绕过失效的 `CloseRequestFcn` 强制删除窗口。每次启动
新演示时也会先自动执行同样的残留窗口清理。

程序只对一个合成的关节速度输入进行积分，并假设模拟关节完美跟随目标。输出目录包含：

```text
summary.json
config_snapshot.json
signals.csv
```

`signals.csv` 中的 `KeyboardReleasePending` 表示键盘释放正在等待确认；`summary.json` 另外
保存 `KeyboardAutoRepeatReleaseCountSuppressed` 和 `KeyboardReleaseCountConfirmed`，便于判断
本机键盘驱动是否产生了自动重复伪释放。

该窗口不构造 `RobotAdapter` 或 `ForceSensorAdapter`，也不调用 `ServoJ/MovJ/MovL`。

## 安全边界

键盘和鼠标可能因为窗口失焦或操作系统事件延迟而丢失释放事件，因此只允许用于：

- 状态机人工观察；
- 离线算法调试；
- 后续 live dry-run 的软件操作请求。

它们不能单独授权真实机器人运动。后续如果增加脚踏开关或手柄按钮，只需通过
`EnableHandleAdapter.submit()` 提交新鲜布尔量和时间戳，状态机与积分器不需要重写。

键盘 50 ms 释放确认只改善离线界面的稳定性，不属于正式 deadman 安全能力；实机输入不得
依赖 GUI 键盘事件。对于最终 `KeyRelease` 丢失，键盘自动重复开始后还使用 200 ms
重复按下 watchdog：重复事件停止即强制撤销软件使能。该机制同样只用于 dry-run；
复用这一延时滤波策略。
