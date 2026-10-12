# 统一关节 PID 控制器

所有正常运动入口使用 `configs/control.json` 的 `shared-joint-pid-v1`，频率 60 Hz。
主从跟随、手动、回零、圆轨迹、有限关节轨迹、Push-T 和抓取共用 `PositionFeedback`。
真机由 `ManualControl` 执行，MuJoCo 由 `SimulationControl` 执行。
场景、路径规划、主从角度映射负责生成目标，不再选择另一套电机控制器。

![统一 PID 控制框图](figures/shared-pid-control.png)

可下载 [SVG](figures/shared-pid-control.svg) / [PDF](figures/shared-pid-control.pdf)。

## 闭环结构

白臂是无主动驱动的输入设备。其独立标定将编码器读数转换为角度，
主从映射将扳机的 0–118° 映射至黑臂夹爪的 0–90°，目标变化率保持 0.6 rad/s。
黑臂六个电机各运行一个位置 PID 外环，并由已有舵机内部位置控制执行命令。
所有内部计算使用标定后的 rad，图表转换为 deg。

每个关节在第 k 次控制更新时：

```text
e       = deadband(q_ref - q)
e_f     = lowpass(e, tau_error)
omega_f = lowpass((q[k] - q[k-1]) / actual_dt, tau_D)
I_next  = I + Ki * e_f * dt
u       = Kp * e_f + I - Kd * omega_f
q_cmd   = q_ref + bounded_correction(u)
```

`q_ref` 是名义位置前馈。D 对测量微分，目标阶跃不会产生微分冲击。
`I` 的单位为 rad；`Kp` 无量纲，`Ki` 为 1/s，`Kd` 为 s。
积分在补偿幅值或物理边界饱和且误差继续推向边界时冻结，反向误差允许积分释放。
输出补偿另有变化率限制。相邻轨迹的正常驻留保留已学到的负载补偿；
重新使能、显式保持、故障和复位会清空相关 PID 状态。
超过 100 ms 的单次控制间隔冻结积分并清除速度估计，补偿积分步长最多 50 ms。
通信看门狗仍独立工作；恢复通信后需要明确恢复控制。

| 关节 | Kp | Ki (1/s) | Kd (s) | 补偿上限 (rad) | 补偿变化率 (rad/s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| shoulder_pan | 0.20 | 1.50 | 0.02 | 0.12 | 0.24 |
| shoulder_lift | 0.25 | 1.50 | 0.02 | 0.12 | 0.24 |
| elbow_flex | 0.25 | 1.50 | 0.02 | 0.12 | 0.24 |
| wrist_flex | 0.25 | 1.50 | 0.02 | 0.12 | 0.24 |
| wrist_roll | 0.25 | 1.50 | 0.02 | 0.12 | 0.24 |
| gripper | 0.50 | 0.80 | 0.01 | 0.05 | 0.04 |

D 滤波时间常数均为 80 ms；误差滤波为前五轴 80 ms、夹爪 120 ms。
死区为 pan/wrist_flex/wrist_roll 0.0015 rad，lift/elbow/gripper 0.003 rad。
物理编码器边界始终约束最终命令，Push-T 另取 CAD 边界交集。

## 执行器与实现边界

当前 Feetech 接口提供 `Goal_Position`、`Goal_Velocity`、`Acceleration` 和
`Present_Current`，没有独立的 `Goal_Current`。因此没有把参考图中的独立电流 PID
或速度 PID 标成已实现。速度估计用于位置 PID 的微分阻尼。
原生位置环、驱动和机械负载共同构成外环的被控对象。

使能前写入并核对当前角度的保持目标，再开启力矩。所有正常运动入口使用 RAM
Acceleration=40、Goal_Velocity=600 ticks/s，并在退出/释放时恢复使能前值。
**原生 P/I/D、EEPROM、工作模式、限位和保护参数不被这套软件 PID 改写。**
`native_defaults` 中 P=16/I=0/D=32 是上游版本参考值，不能作为现场实际读数。
本次现场 P 为 32/48/80/48/32/32，测试报告记录实际值。

MuJoCo 使用相同的 PID 算法、频率和配置，但内部位置执行器、摩擦、负载和通信延迟
并不等价于真机。所有仿真场景的内部位置执行器统一为 kp=700、kv=12，
由 `simulation_actuator` 配置，力矩上限沿用各场景模型；这些数值不是舵机寄存器值。
只修改编译后的 MuJoCo 模型，不改动标定绑定的原始 MJCF。
不能把仿真阶跃结果当作电机实测结果。
预测分支同时复制物理状态和 PID 状态，防止积分/滤波历史导致预测与执行不一致。
原始编码器电气诊断 `hardware move`/夹爪连通性检测保留其有限步进与保护逻辑；
它们不作为 demo 控制器或本次性能验收入口。

## 响应曲线与复现

```sh
# 本地六轴阶跃仿真
./scripts/lerobot-arm verify motors --backend mujoco --output logs/pid-simulation

# 远端真机六轴小幅往返；先做标定一致性、限位与模型碰撞检查
./scripts/lerobot-arm verify motors --backend hardware --execute --output logs/pid-hardware
```

默认从当前姿态逐轴做 3° 阶跃和返回，每段记录 4 s。允许范围为幅度 1–5°、
记录时间 3–10 s。完整轨迹、60 Hz 编码器与 PID 命令、配置 SHA、原生增益、标定 SHA
保存在 `result.json`、`control.json`、`response.csv`；曲线输出 PNG/SVG/PDF。
退出或异常时先保持机械臂，再下载/分析数据。

每个方向分别统计 10–90% 上升时间、到达 90% 时间、超调、±0.5° 调节时间、
最后 0.75 s 的稳态误差及全段 RMS。未达到的指标保留 null，不能用 0 代替。
验收条件：到达 90%、3 s 内进入并保持 ±0.5°、超调不超过 35%、反馈最大间隔不超过 100 ms。
这是当前姿态小阶跃的工程验收，不代表整个工作空间、所有负载或光学末端精度。
白臂是被动输入，六条电机响应对应黑臂六个受控通道。

## 部署

修改唯一参数源后运行 `./scripts/deploy_code.sh`，然后重启两端进程。
握手校验配置 SHA-256；旧进程或旧参数不能静默混用。
`configs/follow.json` 仅保留角色、标定、扳机映射和初始化参数；旧的控制器/频率覆盖字段会被拒绝。

参考：上游 [SO follower](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py)、
[Feetech 寄存器表](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py)、
[ST3215 文档](https://www.waveshare.com/wiki/ST3215_Servo)。
