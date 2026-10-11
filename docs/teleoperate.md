# SO-101 Leader / Follower 遥操作

这一步实现的是：**手动移动 Leader，Follower 跟随它的关节动作**。
电脑负责读取 Leader 的角度，再把目标角度发给 Follower。
它适合验证两臂跟随、手动搬运物体和手动推动 T 块；不会自动规划 Push-T，
也不会保存训练数据或证明末端达到 1 mm 精度。

本项目新增的 `teleoperate` 是启动与配置检查入口，实际控制循环使用固定版本
LeRobot 0.6.1 的 `lerobot-teleoperate`，通过同一 Python 环境的
`python -m lerobot.scripts.lerobot_teleoperate` 启动。
它与 `view arm --hardware-control` 的键盘控制是两个独立流程。

## 1. 需要具备什么

- 一台 Linux x86_64 / Ubuntu 电脑，Python 3.12，并已按 [安装步骤](../README.md#3-configure-and-deploy-the-remote-host)
  安装本项目锁定的 `lerobot[feetech]==0.6.1` 硬件环境。
- SO-101 Leader 和 SO-101 Follower，各自有 USB 通信与符合各自硬件规格的供电。
  **两个 USB 都接在运行控制程序的同一台 Ubuntu 电脑上。**
- 两臂均已完成电机 ID 配置：肩部旋转 1、肩部抬升 2、肘部 3、腕部俯仰 4、
  腕部旋转 5、夹爪 6，并各自完成上游 LeRobot 关节标定。
- Ubuntu 用户有串口访问权限，基座固定，活动空间清空。

Windows 或 Mac 可以通过 `ssh lerobot` 登录 Ubuntu 操作。
下面所有代码都在 **Ubuntu 终端、本项目部署目录** 中运行。
这里不需要摄像头、MuJoCo、网页、X11 或 SSH 网页端口转发。
Miniforge/Conda 不是必需的；如果使用它，必须使用安装了相同依赖的 Python 3.12 环境，
并设置 `PYTHONPATH=src`。下面统一使用项目的 uv 环境，避免调用错 Python。

## 2. 明确哪个 USB 是 Leader

先查看端口：

```bash
ls -l /dev/serial/by-id/
```

它只列出 USB 身份，不会让机械臂运动。拔插确认哪条记录属于哪个设备，
再编辑远端 `configs/arms.toml`。使用稳定的 `by-id` 路径；`ttyACM0/1` 编号可能改变。

现有配置中 `follower` 已确认，`arm1` 仍是 `role = "unassigned"`。
**只有确认 `arm1` 对应真正的 Leader 后**，才把 `[arms.arm1]` 下的 role 改为：

```toml
role = "leader"
```

不要只因有第二个 USB 就把它当作 Leader。入口会拒绝未确认角色、同一设备的两个别名，
以及与上游标准 ID 1～6 不一致的配置。无需修改 `configs/control.json`。
关闭其他电机控制、Reset、Push-T 或诊断程序；同一串口只能由一个流程控制。
入口会持有本项目的两侧串口锁，直到上游程序退出；锁不能阻止独立的第三方程序。

## 3. 准备两侧的关节标定

如果已有正确标定，继续使用原来的设备 ID，不需要每次重新标定。
ID 是标定文件的名字，必须绑定对应的物理机械臂，不能交换两臂文件。

下面是首次标定示例。先把两个 `YOUR_...` 替换为刚确认的真实 `by-id` 路径，
已有标定时把 `my_so101_...` 替换为原来的 ID。

```bash
LEADER_PORT=/dev/serial/by-id/YOUR_LEADER_USB_ID
FOLLOWER_PORT=/dev/serial/by-id/YOUR_FOLLOWER_USB_ID
CALIBRATION_ROOT="$HOME/.cache/huggingface/lerobot/calibration"

./scripts/uv run --locked --extra hardware lerobot-calibrate \
  --teleop.type=so101_leader --teleop.port="$LEADER_PORT" \
  --teleop.id=my_so101_leader \
  --teleop.calibration_dir="$CALIBRATION_ROOT/teleoperators/so_leader"

./scripts/uv run --locked --extra hardware lerobot-calibrate \
  --robot.type=so101_follower --robot.port="$FOLLOWER_PORT" \
  --robot.id=my_so101_follower \
  --robot.calibration_dir="$CALIBRATION_ROOT/robots/so_follower"
```

这两条命令分别标定 Leader 与 Follower，会写电机标定寄存器。
按照终端提示摆中间姿态、逐个活动关节并记录范围；不要硬推到机械限位。
成功后应分别有：

```text
teleoperators/so_leader/my_so101_leader.json
robots/so_follower/my_so101_follower.json
```

这是关节标定，不是相机内外参，也不是本项目的模型角度标定。
改变电机零点/范围后，本项目以前的角度标定、停靠姿态等记录需要重新检查，
不能直接沿用。官方流程见 [SO-101 文档](https://huggingface.co/docs/lerobot/en/so101)。

## 4. 先检查启动参数

```bash
./scripts/uv run --locked --extra hardware python -m lerobot_arm_control teleoperate \
  --leader arm1 --follower follower \
  --leader-id my_so101_leader --follower-id my_so101_follower \
  --calibration-root "$CALIBRATION_ROOT" --duration 10
```

这条命令**只预览，不打开串口、不让机械臂运动**。
输出两臂名称/端口、标定文件的完整路径及 `present` / `missing`，最后显示真实上游命令。
端口名称取自 `arms.toml`；设备 ID 用于找 JSON 文件，两者作用不同。
缺文件应先解决；执行时还会检查 JSON 的六个关节、ID 与编码器范围。

默认标定根目录遵循 `HF_LEROBOT_CALIBRATION`、`HF_LEROBOT_HOME`、`HF_HOME`、
`XDG_CACHE_HOME`。示例显式传入路径，避免从别的环境读到另一套标定。
`--config /path/to/arms.toml` 可以选择另外一份设备配置。

## 5. 开始跟随

把两臂摆到相近的关节姿态，托稳 Follower，夹爪远离手指和物体，然后运行：

```bash
./scripts/uv run --locked --extra hardware python -m lerobot_arm_control teleoperate \
  --leader arm1 --follower follower \
  --leader-id my_so101_leader --follower-id my_so101_follower \
  --calibration-root "$CALIBRATION_ROOT" \
  --fps 30 --max-relative-target 1 --duration 10 --execute
```

`--execute` 才真正打开设备并使 Follower 跟随。先慢慢活动 Leader 的一个关节，
确认 Follower 对应关节的方向；初次只做 10 秒空载检查。
`--duration 10` 限制跟随循环的时间，不包含连接或交互标定的等待时间。
确认方向、姿态和夹爪跟随正常后，可去掉 `--duration`，持续运行到 `Ctrl+C`。

| 参数 | 作用 |
| --- | --- |
| `--leader / --follower` | 选择 arms.toml 中的设备条目 |
| `--leader-id / --follower-id` | 找到两侧各自的上游标定文件 |
| `--fps 30` | 目标控制循环频率；不是保证达到的串口刷新率 |
| `--max-relative-target 1` | 每条命令相对当前反馈的位置差上限：五个关节 1°，夹爪 1 个百分比点 |
| `--execute` | 从预览切换为真实遥操作 |

两侧五个关节都使用角度单位；夹爪为 0～100 的归一化开合量。
位置差上限用于限制单条目标跳变，**不是末端 1 mm 限制、速度保证或避碰功能**。
跟随仍可能追赶 Leader，因此启动前的姿态应相近。

上游连接会配置电机模式、PID 和夹爪保护参数；不会使用本项目 `control.json` 的补偿、
速度设置或恢复策略。如果文件和电机内的标定不一致，上游可能交互询问是否使用文件，
即使本地 JSON 检查已通过也不代表 EEPROM 一致。

结束时先托稳两臂，再按 `Ctrl+C`。正常结束按上游流程关闭扭矩，Follower 可能下落。
入口会把 Ctrl+C、SSH 挂断 SIGHUP 和终止 SIGTERM 转发为上游中断，并等待它退出；
没有本项目实时控制的 SSH 心跳停机机制，
也不能保证强制杀进程、断电或驱动错误时完成清理。现场断电方法需要保持可用。

## 6. 常见问题和验收

| 现象 | 先检查什么 |
| --- | --- |
| role 为 unassigned，入口拒绝启动 | 物理确认 Leader，然后修改它的 role |
| Missing upstream calibration | 检查 ID 和标定根目录，缺哪侧就标定哪侧 |
| No status packet / Incorrect status packet | 电源、USB、串口是否对应、是否有其他进程占用；不要连续重复启动 |
| 同样姿态却跟随错位 | 两臂的装配方向、各自关节标定及文件与设备是否对应 |
| Windows 下执行失败 | 真正执行必须在两臂 USB 所在的 Ubuntu 主机；Windows 负责 SSH |

代码测试只验证参数、配置检查、串口锁和启动流程，不替代现场验收。
现场依次确认五个关节与夹爪的跟随方向、10 秒退出后的状态，再进行空载持续跟随。
通过后才放入轻小物体做手动搬运/推动；自动 Push-T 与视觉闭环需要另行接入。
