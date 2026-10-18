# 机械臂配置与上电核对

`configs/robot.json` 是当前两臂参数的唯一基准，替代原来的 arms.toml、control.json、follow.json。

- `arms.<name>`：端口、角色、后端、型号、波特率、六关节 ID。
- `resources`：独立六关节标定、模型文件路径及 SHA256，数据仍保存在原文件中。
- `parameters.<joint>.persistent`：所有可写 EEPROM 参数，包括 PID、死区、零偏、限位、模式、保护阈值。
- `startup`：加速度、目标时间/速度的零值及输出限制。
- `identity`：型号、固件、ID、波特率、预留/工厂字段；只检查，不猜测新 ID 或修改厂家只读字段。
- `control`：共享控制器、运行期原生参数覆盖、轨迹和保护配置。
- `follow`：角色映射、扳机端点与初始化配置。

这是 2026-10-19 双臂实读保存的配置。相机和桌面属于环境配置，继续单独保存。
其他机器人必须使用自己的标定和参数；本配置不是所有 STS3215 的通用出厂值。
私有 `configs/robot.local.json` 只能覆盖 USB `port`，Git 和部署均忽略该文件。

## 自动检查和恢复

所有实际串口入口都执行同一核对：普通连接及工作台发现连接。
因此跟随、复位、画圆、Push-T、Pick-T、标定、读数和诊断均受此检查；纯仿真不访问舵机。
工作台只连接到部分电机时检查在线部分，报告缺失轴，不把缺失电机视为通过。
工作台使用不同 ID/波特率前必须先更新配置，避免给另一个电机套用标定。

在远端运行：

```bash
sh scripts/install_config_guard.sh
PYTHONPATH=src .venv/bin/python -m lerobot_arm_control config check
PYTHONPATH=src .venv/bin/python -m lerobot_arm_control config check --arm follower
journalctl --user -u lerobot-config-guard -n 30
```

守护服务每 4 秒重新读取配置，USB/电源恢复后自动重试；通过串口独占锁避让控制进程。
普通连接仍会独立检查，不依赖守护服务已经完成。用户服务安装时启用 linger，使退出登录后继续运行。
另一进程占用串口、硬件掉线或配置不合法时会记录原因并重试，不会绕过检查启动控制。

完全一致时不写入 EEPROM。只有确实不同的字段才进入写入、回读、失败回滚和重新加锁的事务。
恢复要求所有在线轴扭矩关闭、静止、没有非电压故障，温度和供电处于保存的写入范围。
发现承重中的机械臂参数不同，会拒绝控制并等待手托并关闭扭矩；不会擅自卸力。
此时工作台仍允许查看、保持和显式释放。托好机械臂并释放后，点击清除故障会重新核对恢复，
若恢复改变了参考系或 EEPROM 限位，则需重新设置受影响轴的动作范围再使能；仅恢复 PID 不会清除范围。
不会恢复目标位置、扭矩使能、反馈状态或预留/厂家字段，也不会执行原点重置指令。
ID/波特率/固件/工厂数据不一致会报错，需要核对接线或设备身份。
Phase 恢复后记录持久的待重启状态，后续连接也会拒绝控制，避免只读回寄存器就误认为已生效。
完成实体断电重启后，显式执行 `config confirm-power-cycle --arm ARM_NAME`；
该确认要求所有轴已释放、静止且 Phase 回读匹配。电机没有可用的重启计数器，因此这一特殊情况
需要操作者确认，不能依靠反复重连自动解除。普通 PID、零偏和限位差异不需要此确认。

肩、肘 EEPROM I 基准仍为 4、8；正常控制由共享控制器临时写 I=0，退出恢复。
守护服务不抢占控制中的串口，因此不会覆盖临时参数。控制参数 SHA256 对应
`control` 子树规范化 JSON，保持此前已验证轨迹的控制指纹，不受端口或快照时间变化影响。

## 有意修改后保存

工作台仍可修改参数；退出后若未更新基准，下次连接或守护检查会恢复旧值。
先停止守护，再在工作台完成修改、断开工作台连接，然后显式保存：

```bash
systemctl --user stop lerobot-config-guard
PYTHONPATH=src .venv/bin/python -m lerobot_arm_control config capture --arm follower
PYTHONPATH=src .venv/bin/python -m lerobot_arm_control config check --arm follower
systemctl --user start lerobot-config-guard
```

capture 不发运动指令，要求所有轴在线、静止且 EEPROM 已锁定。临时控制必须先退出，
不可把控制中临时 I/速度误存成新基准。参考系、限位、ID 或模式有意变化时，必须先重新核对
对应标定及连接信息，并在 robot.json 中一起更新；仅保存 PID 不需要重新标定。
共享参数修改后需同步到本地/远端，私有 USB 路径各机器自行配置。

## RobotArm

```python
from lerobot_arm_control.robot import RobotArm

arm = RobotArm.from_name('follower')
with arm.connect(allow_alarm=True):
    print(arm.startup_report)  # checked / missing / restored
    controller = arm.create_controller(calibration, raw_limits)
    # 沿用 ManualControl 的现有 enable/目标/close 协议。
    # 每次启用运动仍受原有范围、碰撞、受阻和反馈保护限制。
```

现有入口继续可用，SDK bus 的 `robot_arm` 属性指向同一对象。
ManualControl 与工作台会绑定为 `arm.controller`；配置、连接及控制器归属于同一机械臂。
新硬件后端可在 `MotorBackend` 中注册 `ensure_parameters(bus, arm)` 适配器；
配置提供了硬件基准但后端没有适配器时直接报错，不静默跳过。

## 本次验收（2026-10-19）

双臂十二轴均在线，完整基准核对无差异。白臂扳机在六轴扭矩关闭的情况下，
实测将 P 从 16 临时改为 17、RAM 加速度从 0 改为 1，新连接自动恢复为 16/0。
前后所有六轴的编码器、目标位置和扭矩使能读数完全一致，没有发送运动指令。

后台守护测试中，持有串口 6 秒时临时参数保持不变；释放串口后约 3.04 秒由守护恢复。
服务已启用开机启动及 linger。验证的是实际参数差异恢复与串口互斥；操作者随后确认已做好支撑并实际断电重上电，十二轴再次实读一致且 Status=0。
普通连接及真实工作台入口均通过，十二个电机各 58 个寄存器全部读取成功。
黑臂读到扭矩使能、白臂读到扭矩关闭，核对过程保留实际状态，不假定上电必然关闭扭矩。
Phase 等需重新上电生效的字段没有进行实机改写。
