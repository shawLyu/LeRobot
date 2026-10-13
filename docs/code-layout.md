# 代码分类与配置归属

本项目直接从源码运行，统一入口是 `./scripts/lerobot-arm`，无需打包安装。
本地 checkout 是代码来源；远端只部署 `configs/remote-runtime.txt` 列出的运行模块。

```text
LeRobot/
├── src/lerobot_arm_control/
│   ├── core/             配置、路径、文件、进程命令与延迟加载 API
│   ├── robot/so101/      LeRobot 总线适配、设备身份与模型元数据
│   ├── control/          共享 PID、限位、保持、接触/受阻检测、轨迹执行
│   ├── calibration/      独立关节标定及相机、标记、桌面标定
│   ├── perception/       桌面注册、双臂基座定位、物体检测与跟踪
│   ├── planning/         本地 MuJoCo 规划、IK 与碰撞验证
│   ├── tasks/
│   │   ├── circle/       圆轨迹生成与执行
│   │   ├── push_t/       推 T 任务、快照与本地/远端执行适配
│   │   └── pick_t/       抓放任务与阶段编排
│   ├── transport/        协议、连接、最新状态发布与只读订阅
│   ├── runtime/          本地客户端、远端 worker、仿真 worker
│   │   └── common.py     worker 共用定时、stdio 通信与延后日志写入
│   ├── visualization/    本地窗口、场景重建与外观
│   ├── diagnostics/      响应曲线、跟踪指标和精度验收
│   ├── cli.py            命令路由
│   └── __main__.py       python -m lerobot_arm_control
├── configs/              运行配置、场景参数与远端部署清单
├── scripts/              启动、安装、部署、硬件检查与 X11 相机入口
├── tests/                离线回归测试；其中一部分随远端运行代码部署
├── assets/               机器人模型、网格、许可证及场景资源
├── data/                 标定、轨迹、桌面注册和实验数据
├── docs/                 使用说明、控制设计与测量报告
└── logs/                 运行日志与控制响应记录（默认忽略）
```

## 模块边界

`control/position_feedback.py` 实现六通道共享 PID；`ManualControl` 管理真机反馈、
启用、运行范围、故障锁存、保持和原生 RAM 参数恢复。仿真控制器运行同一 PID 算法。
手动、复位、circle、有限轨迹、follow、Push-T 和 pick/place 使用这些控制适配器。
任务模块负责目标和阶段，不维护另一套电机 PID。

白臂采样、独立标定、映射、PID 和黑臂写入全部在远端
`runtime/follow_server.py --standalone` 中完成。本地 simulator 通过只读状态流显示。
`runtime/common.py` 提供公共 worker 工具；具体 worker 不再从另一个 worker 导入它们。
规划、MuJoCo 和图形窗口留在本地；远端导入闭包由测试校验。

## 参数放在哪里

| 文件/目录 | 负责的参数 | 修改方式 |
| --- | --- | --- |
| `configs/runtime.json` | 默认 SSH 主机、远端项目目录、本地备用 Python 环境 | 编辑配置；SSH 可由 CLI 或环境变量覆盖 |
| `configs/arms.toml` | 两臂角色、USB 设备、波特率、电机 ID | 每台设备单独配置；部署保留已有文件 |
| `configs/control.json` | 唯一电机控制频率、PID、参考整形、轨迹速度、接触/受阻阈值及绑定设备的指令范围 | 修改后部署并重启；控制连接核对 SHA256 |
| `configs/follow.json` | 主从映射、扳机端点、初始化策略及验收速度范围 | 实测确认后保存；不重复定义 PID |
| `configs/camera.toml`、`camera_intrinsics.json` | 摄像头选择、采集设置、内参 | 设备配置与内参标定分别维护 |
| `configs/tabletop.json` | 标记尺寸、高度、桌面与视觉处理参数 | 按实测值更新；常规部署保留远端已有文件 |
| `configs/scenes/tabletop-current.json` | 当前场景参数及配置/标定来源哈希 | 场景更新时保存 |
| `data/calibration/` | 两臂独立角度、静止姿态、基座与桌面参考标定 | 标定流程生成；保留原始测量和身份绑定 |
| `configs/remote-runtime.txt` | 远端所需源文件、脚本、资源与测试 | 模块依赖变化后更新并校验闭包 |

连接设置优先级为 **显式参数 → 环境变量 → runtime.json**。
本地/远端客户端统一接受 `--host` 和 `--remote-root`；部署和 X11 脚本使用
`LEROBOT_HOST`、`LEROBOT_REMOTE_ROOT`。本地启动器优先使用 `LEROBOT_MUJOCO_ENV`，
其次是项目 `.venv`，Mac 最后读取 `local.fallback_environment`。
需要切换资源目录时使用 `--workspace` 或 `LEROBOT_ARM_WORKSPACE`；选中的 workspace
必须提供完整配置，缺失时直接报错，不借用另一台机器人的参数。

当前共享控制频率为 60 Hz；原生 RAM 加速度 254、目标速度 3000 ticks/s 在退出时恢复；
圆轨迹默认周期为 16 秒。详细 PID 和保护策略见 [控制设计](lerobot-control.md)。
这些数值以配置文件为准，不在 demo 中重复写死。

共享保护逻辑的真实夹持验证仍待完成；离线测试不能替代它。

## 部署与数据

```sh
./scripts/deploy_code.sh --dry-run
./scripts/deploy_code.sh
```

部署传输清单内的源码、测试，以及公共配置和文档；保留设备配置、
标定、环境和日志。增量同步不会删除清单之外的远端文件，也不会生成旧代码归档。

版本控制包含源码、测试、配置、接受的标定、文档和必要模型。
生成的场景、网格副本、日志、数据集、缓存与恢复快照通常被忽略。
不要通过 `git clean -fdx` 清理实验资料；保留规则见 [数据说明](../data/README.md)。
环境和操作入口见 [系统结构与运行](system.md)，跟随操作见 [Follow](follow.md)。
