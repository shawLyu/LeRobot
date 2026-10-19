# 代码分类与配置归属

本项目直接从源码运行，统一入口是 `./scripts/lerobot-arm`，无需打包安装。
本地 checkout 是代码来源；远端只部署 `configs/remote-runtime.txt` 列出的运行模块。

```text
LeRobot/
├── src/lerobot_arm_control/
│   ├── core/             配置、路径、文件、进程命令与延迟加载 API
│   ├── robot/
│   │   ├── model.py      通用标量关节 MJCF 元数据
│   │   ├── backends.py   驱动注册、配置与型号身份接口
│   │   ├── arm.py        RobotArm：配置、连接、控制器归属
│   │   ├── parameters.py 保存基准、差异恢复与 EEPROM 事务
│   │   ├── config_cli.py 参数核对、保存与上电守护
│   │   ├── connection.py 串口独占与统一配置核对
│   │   ├── feetech.py    STS3215 驱动和原生流参数校验
│   │   └── so101/        SO-101 关节表和工具几何选择
│   ├── control/          共享 PID、限位、保持、接触/受阻检测、轨迹执行
│   ├── calibration/      三类标定入口：table / camera / angles；高级拟合归入 angles
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
│   ├── workbench/        六电机监测、原生参数事务、位置控制和浏览器工作台
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

`control/position_feedback.py` 按传入关节和限位实现共享 PID，不依赖通信协议或
六关节名称。`control/factory.py` 定义策略接口和显式注册/创建入口；`ManualControl` 管理真机反馈、
启用、运行范围、故障锁存、保持和原生 RAM 参数恢复。仿真控制器通过同一工厂选择策略，并允许显式传入其他关节表、控制频率和执行器设置。
手动、复位、circle、有限轨迹、follow、Push-T 和 pick/place 使用这些控制适配器。
任务模块负责目标和阶段，不维护另一套电机 PID。

机械模型元数据与电机型号相互独立；`ArmConfig.backend` 默认 `feetech`。
`robot/connection.py` 按后端验证配置及 ping 型号，不再属于某一个机械臂结构。
STS 原生寄存器工作台仍明确限定 Feetech STS3215，范围存储归属
`workbench/ranges.py`，限位与运动检查归属 `workbench/controller.py`。
接入步骤、控制器契约和当前扩展边界见 [扩展指南](extending.md)。

白臂采样、独立标定、映射、PID 和黑臂写入全部在远端
`runtime/follow_server.py --standalone` 中完成。本地 simulator 通过只读状态流显示。
`runtime/common.py` 提供公共 worker 工具；具体 worker 不再从另一个 worker 导入它们。
规划、MuJoCo 和图形窗口留在本地；远端导入闭包由测试校验。

文件指纹统一调用 `core/files.py` 的 `file_digest`，文件内容仍按原始字节计算 SHA256；
计划等结构化数据保留各自协议规定的 JSON 哈希格式。原生舵机阶跃指标集中在
`diagnostics/response_metrics.py`，命令行验收和工作台调用同一实现；工作台不再依赖
原生 PID 验收命令入口。进程中断处理归属 `core/processes.py`。

## 参数放在哪里

| 文件/目录 | 负责的参数 | 修改方式 |
| --- | --- | --- |
| `configs/runtime.json` | 通用连接默认值，不包含个人账户和环境路径 | 共享示例 |
| `configs/runtime.local.json` | 当前账户的 SSH、远端目录与可选本地环境 | Git 忽略、部署排除；每台机器独立维护 |
| `configs/robot.json` | 两臂连接、舵机基准、标定/模型指纹、共享控制器与主从映射 | 唯一机械臂配置；部署同步，上电与每次连接检查 |
| `configs/robot.local.json` | 机器私有 USB 端口覆盖 | Git 忽略、部署排除 |
| `configs/camera.toml`、`camera_intrinsics.json` | 摄像头选择、采集设置、内参 | 设备配置与内参标定分别维护 |
| `configs/tabletop.json` | 标记尺寸、高度、桌面与视觉处理参数 | 按实测值更新；常规部署保留远端已有文件 |
| `configs/scenes/tabletop-current.json` | 当前场景参数及配置/标定来源哈希 | 场景更新时保存 |
| `data/calibration/` | 两臂独立角度、静止姿态、基座与桌面参考标定 | 标定流程生成；保留原始测量和身份绑定 |
| `configs/remote-runtime.txt` | 远端所需源文件、脚本、资源与测试 | 模块依赖变化后更新并校验闭包 |

连接设置优先级为 **显式参数 → 环境变量 → runtime.local.json → runtime.json**。
本地/远端客户端统一接受 `--host` 和 `--remote-root`；部署和 X11 脚本使用
`LEROBOT_HOST`、`LEROBOT_REMOTE_ROOT`。本地启动器优先使用 `LEROBOT_MUJOCO_ENV`，
其次是项目 `.venv`，Mac 最后读取 `local.fallback_environment`。
需要切换资源目录时使用 `--workspace` 或 `LEROBOT_ARM_WORKSPACE`；选中的 workspace
必须提供完整配置，缺失时直接报错，不借用另一台机器人的参数。

当前共享控制频率为 60 Hz。默认参数档保留 RAM 加速度 254；水平高速圆显式选择
`smooth-circle-v1`，以相同控制算法使用速度前馈和 RAM 加速度 16。
运行期加速度和目标速度寄存器在退出时恢复。原始小圆周期仍为 16 秒；
已验证的 8 cm 水平圆周期为 6 秒、匀速 60°/s，启停各渐变 6 秒。
另有 34/36/38 cm 高度的 90°/s 实验计划，显式选择 `smooth-circle90-v2`，进入/低速启停加速度 16，随速度平滑提高到匀速 32；
多圈时保留 6 秒启停并拒绝重叠。各速度档共享控制器算法，配置覆盖关系明确。
120°/s 使用单独的 38 cm 路径和 `smooth-circle120-v1`，原生 RAM 加速度为
16→48→16，仍受原有关节速度/加速度预算约束；34/36 cm 不满足该速度的预算。
`tasks/circle/geometry.py` 统一验证圆平面的正交基，供远端身份/参数检查、
本地几何检查和响应投影共用；15°/30° 倾斜场景使用相同控制器和速度档。
见 [圆轨迹运行与测量](circle-trajectory.md)；详细 PID 和保护策略见 [控制设计](lerobot-control.md)。
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
