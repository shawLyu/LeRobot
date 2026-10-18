# 接入机器人模型、控制器和硬件驱动

模型描述、目标控制算法和电机总线分别扩展。现有入口继续默认使用 SO-101、
Feetech STS3215 和共享 PID；无需改写设备配置、已保存行程或 PID 参数。
扩展注册必须在应用启动时显式完成，配置文件中的字符串不会自动导入或执行插件代码。

## 模块与责任

| 接口 | 所属模块 | 负责什么 |
|---|---|---|
| `RobotModel.from_mjcf` | `robot/model.py` | 关节名称、顺序、显式弧度限位；不连接设备 |
| `MotorBackend`、`register_backend` | `robot/backends.py` | 总线创建、设备配置校验、型号编号 |
| `connected` | `robot/connection.py` | 独占连接、身份核对、只读会话退出；不自动使能或配置电机 |
| Feetech 适配器 | `robot/feetech.py` | SDK 构造、STS3215 配置和原生 RAM 参数校验 |
| `JointController`、`register_controller`、`create_controller` | `control/factory.py` | 按配置创建相互独立的控制算法实例 |
| `PositionFeedback` | `control/position_feedback.py` | 任意数量标量转动关节的有界 PID、参考整形和状态 |
| `ManualControl` | `control/manual_control.py` | 现有 SO-101 真机使能、保持、夹持与受阻保护、标定和写入 |
| `SimulationControl` | `control/simulation_control.py` | 将选定策略应用到 MuJoCo 位置执行器 |
| 范围文件读写 | `workbench/ranges.py` | 私有范围持久化；不访问总线或执行运动 |

任务代码生成目标和阶段；运行入口组织连接及生命周期；控制器计算目标；
适配器负责实际写入、硬件边界和保护。新增算法不要复制 demo 的串口或安全代码。

## 其他 MJCF 模型

`RobotModel` 读取单个 MJCF 中显式声明的 hinge 关节名称和范围，要求
`<compiler angle="radian"/>`。它不展开 include、继承默认关节范围或解释 free/ball
关节。复杂模型可由 MuJoCo 编译后明确指定参与控制的关节名称，不要推测关节顺序。

```python
import mujoco

from lerobot_arm_control.robot import RobotModel
from lerobot_arm_control.control.simulation_control import SimulationControl

spec = RobotModel.from_mjcf("my_robot.xml")
model = mujoco.MjModel.from_xml_path(str(spec.path))
data = mujoco.MjData(model)
profile = {
    "version": 1,
    "joints": {
        joint: {
            "kp": 0.4,
            "ki": 0.5,
            "max_correction_rad": 0.1,
            "correction_rate_rad_s": 0.1,
        }
        for joint in spec.joints
    },
}
control = SimulationControl(
    model, data,
    joints=spec.joints,
    feedback_profile=profile,
    control_hz=50,
    actuator_settings={"kp": 100, "kv": 10},
)
control.step([0.0] * len(spec.joints))
```

每个选定关节必须有且只有一个对应的位置执行器，并声明正确的 `ctrlrange`。
请为新模型选择合适的惯量、阻尼、仿真步长和执行器参数；示例数值不代表新模型已调好。
`SimulationControl` 不修改源 MJCF。它限制输出到执行器范围，拒绝非有限输出，
按配置频率运行控制器，且在时间回退时清除历史状态。rollout 复制不共享 PID 内存，
并要求相同关节、范围和采样频率。

现有 SO-101 CLI、主从跟随、标定、规划和六通道工作台仍有各自的机械结构/协议约束。
上述接口使控制核和仿真适配器可复用，不表示改一个文件名即可运行任意机器人。
这些 SO-101 专用入口中的 `JOINTS` 与包顶层导出的默认关节表继续保持兼容。

## 替换控制策略

控制器工厂接收 `(profile, limits)`，返回符合 `JointController` 的实例：

```python
from lerobot_arm_control.control import register_controller

class ReferenceController:
    def __init__(self, profile, limits):
        self.profile = profile
        self.limits = limits

    def reset(self, joints=None):
        pass

    def finish_reference(self, joints):
        pass

    def target(self, joint, reference, measured, elapsed):
        low, high = self.limits[joint]
        return max(low, min(high, reference))

    def status(self):
        return {}

register_controller("reference", ReferenceController)
```

这是接口示例，没有 PID、参考平滑或负载补偿，不是建议替换已调好的真机策略。
在仿真中选择：

```python
control = SimulationControl(
    model, data, joints=spec.joints,
    feedback_profile={"controller": "reference"},
)
```

`ManualControl(..., feedback_profile=...)` 使用同一个工厂入口。
省略配置中的 `controller` 时始终选择内置 `pid`，继续读取原有共享参数。
未知名称和重复注册会报错，不会静默退回另一种策略。每次创建会复制 profile，
避免修改共享配置或另一个控制器实例。控制器应使用实际 `elapsed`、提供状态，
并在 `reset`/`finish_reference` 中明确处理其积分、滤波和预测历史。
硬件适配器的校准限位、看门狗、接触/受阻检测和保持生命周期继续生效。

## 其他电机总线

`configs/arms.toml` 中新增可选 `backend` 字段。省略时仍为 `feetech`；`model`
继续表示电机型号，不是机器人 MJCF 型号。内置驱动仍只接受已经验证的 STS3215、
1 Mbps、稳定 USB 路径和 1–253 ID。其他硬件需要自己的验证及身份表。

应用入口在 `load_arms` 之前注册适配器：

```python
from lerobot_arm_control.robot import MotorBackend, register_backend
from my_robot_driver import make_bus, validate_arm

register_backend("my_bus", MotorBackend(
    factory=make_bus,
    validate=validate_arm,
    model_numbers={"my_servo": 1234},
))
```

```toml
[arms.other]
backend = "my_bus"
model = "my_servo"
port = "/dev/serial/by-id/YOUR_DEVICE"
role = "follower"
baudrate = 115200

[arms.other.motors]
base = 1
wrist = 2
```

`1234` 和波特率仅为接口示例，必须替换成真实驱动验证的值。
配置加载只校验，不调用总线工厂。工厂创建总线但不打开设备、不使能扭矩、
不改写 EEPROM。当前连接管理器面向兼容 LeRobot 语义的串口总线：
`connect(handshake=False)`、`ping`、`port_handler.ser.exclusive`、
`is_connected` 和 `disconnect(disable_torque=False)`；带报警的只读连接还需要
对应 packet handler 接口。非兼容网络/CAN 接口需要自己的连接生命周期适配，
不要伪装成 Feetech 寄存器表。

新的后端不自动获得 SO-101 的完整真机任务支持。还需明确反馈单位、标定、
关节顺序、可写寄存器、故障语义和运动保护。STS3215 原生参数工作台会在打开
其他型号总线之前拒绝操作，避免使用错误的寄存器地址。既有 SO-101 真机入口和
原生 PID 工具也通过 `connected_sts3215` 明确校验这一边界；通用的 `connected`
接口才按注册后端连接。

## 验证与部署

`tests/test_extensions.py` 使用替代总线和控制器验证扩展点，包含非六轴 PID、
实例隔离、身份不匹配、未知插件和原生寄存器工具的适配边界。
`tests/test_simulation_control.py` 实际运行独立两关节 MuJoCo 模型，验证频率、
替代策略限幅和非法输出。既有 SO-101、PID、保护策略和范围文件回归继续执行。
这些测试不会驱动真实机械臂。

新增远端模块要加入 `configs/remote-runtime.txt`；测试会检查导入闭包，
确保远端无需导入 MuJoCo 或图形库，普通导入也不会加载硬件 SDK。
外部扩展模块同样需要在两端部署，并在各自入口注册，随后再运行各自设备的验收。
源代码导入入口 `robot/so101/driver.py` 已合并到 `robot/connection.py`；
公开的 `from lerobot_arm_control.robot.so101 import connected, read_state` 保持可用。
