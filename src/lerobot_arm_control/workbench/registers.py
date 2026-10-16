"""Documented STS TTL registers, with explicit widths, access and raw units.

Source: https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/
Reserved bytes are not writable. Values are device register integers, not SI gains.
"""

from dataclasses import asdict, dataclass

BAUDRATES = (1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400)


@dataclass(frozen=True)
class Register:
    key: str
    address: int
    size: int
    label: str
    group: str
    minimum: int = 0
    maximum: int = 254
    unit: str = 'raw'
    note: str = ''
    effect: str = ''

    @property
    def writable(self):
        return self.group in ('EEPROM', 'RAM')

    def check(self, value):
        if not self.writable:
            raise ValueError(f'{self.key} 是只读参数')
        if type(value) is not int or not self.minimum <= value <= self.maximum:
            raise ValueError(f'{self.key} 必须是 {self.minimum}..{self.maximum} 的整数')


# SDK spelling is retained where correct, so sign encoding remains device-specific.
REGISTERS = (
    Register('Firmware_Major_Version', 0, 1, '固件主版本', 'Identity'),
    Register('Firmware_Minor_Version', 1, 1, '固件次版本', 'Identity'),
    Register('Endianness', 2, 1, '字节序', 'Identity', note='0 = 小端'),
    Register('Model_Number', 3, 2, '型号编号', 'Identity'),
    Register('ID', 5, 1, '舵机 ID', 'EEPROM', 1, 253, effect='transport',
             note='只允许单个在线电机修改；重新连接时使用新 ID'),
    Register('Baud_Rate', 6, 1, '波特率编号', 'EEPROM', 0, 7, effect='transport',
             note='0:1M / 1:500k / 2:250k / 3:128k / 4:115200 / 5:76800 / 6:57600 / 7:38400'),
    Register('Return_Delay_Time', 7, 1, '地址 7（SDK 返回延时）', 'Identity',
             note='厂商 STS 表标为预留，仅显示，不写入'),
    Register('Response_Status_Level', 8, 1, '应答状态级别', 'EEPROM', 0, 1,
             note='0 = 仅读和 Ping 应答；1 = 所有指令应答'),
    Register('Min_Position_Limit', 9, 2, '最小位置限制', 'EEPROM', 0, 4094, 'tick',
             effect='reference'),
    Register('Max_Position_Limit', 11, 2, '最大位置限制', 'EEPROM', 0, 4095, 'tick',
             effect='reference'),
    Register('Max_Temperature_Limit', 13, 1, '最高温度', 'EEPROM', 0, 100, '°C'),
    Register('Max_Voltage_Limit', 14, 1, '最高电压', 'EEPROM', 0, 254, '0.1 V'),
    Register('Min_Voltage_Limit', 15, 1, '最低电压', 'EEPROM', 0, 254, '0.1 V'),
    Register('Max_Torque_Limit', 16, 2, '上电最大输出限制', 'EEPROM', 0, 1000, '0.1%'),
    Register('Phase', 18, 1, '舵机相位位掩码', 'EEPROM', effect='reference',
             note='方向、速度单位、驱动桥等；部分位需重启生效'),
    Register('Unloading_Condition', 19, 1, '保护卸载位掩码', 'EEPROM',
             note='bit0 电压 / bit1 编码器 / bit2 温度 / bit3 电流 / bit5 过载'),
    Register('LED_Alarm_Condition', 20, 1, 'LED 报警位掩码', 'EEPROM'),
    Register('P_Coefficient', 21, 1, '位置环 P', 'EEPROM', note='厂商系数，比例缩放 1/4'),
    Register('D_Coefficient', 22, 1, '位置环 D', 'EEPROM', note='厂商系数，比例缩放 1/8'),
    Register('I_Coefficient', 23, 1, '位置环 I', 'EEPROM'),
    Register('Minimum_Startup_Force', 24, 1, '最小启动输出', 'EEPROM', unit='0.1%'),
    Register('Integral_Limit', 25, 1, '积分限制', 'EEPROM', note='上限 = 数值 × 4；0 = 不限制'),
    Register('CW_Dead_Zone', 26, 1, '正向死区', 'EEPROM', 0, 16, 'tick'),
    Register('CCW_Dead_Zone', 27, 1, '反向死区', 'EEPROM', 0, 16, 'tick'),
    Register('Protection_Current', 28, 2, '保护电流', 'EEPROM', 0, 500, '6.5 mA',
             note='厂商标注可设置最大值为 500'),
    Register('Angular_Resolution', 30, 1, '角度分辨率倍数', 'EEPROM', 1, 128,
             effect='reference'),
    Register('Homing_Offset', 31, 2, '位置偏移', 'EEPROM', -2047, 2047, 'tick',
             note='SDK 标准符号位编码范围', effect='reference'),
    Register('Operating_Mode', 33, 1, '运行模式', 'EEPROM', 0, 3, effect='reference',
             note='0 位置 / 1 恒速 / 2 PWM / 3 步进；本界面运动测试使用位置模式 0'),
    Register('Protective_Torque', 34, 1, '保护保持输出', 'EEPROM', unit='1%'),
    Register('Protection_Time', 35, 1, '过载保护时间', 'EEPROM', unit='10 ms'),
    Register('Overload_Torque', 36, 1, '过载输出阈值', 'EEPROM', unit='1%'),
    Register('Velocity_closed_loop_P_proportional_coefficient', 37, 1,
             '恒速模式 P', 'EEPROM'),
    Register('Over_Current_Protection_Time', 38, 1, '过流保护时间', 'EEPROM', unit='10 ms'),
    Register('Velocity_closed_loop_I_integral_coefficient', 39, 1,
             '恒速模式 I', 'EEPROM'),
    Register('Torque_Enable', 40, 1, '扭矩使能', 'RAM', 0, 2,
             note='0 释放 / 1 位置使能 / 2 阻尼；不提供写 128 的原点重置捷径'),
    Register('Acceleration', 41, 1, '加速度', 'RAM', unit='8.7 °/s²',
             note='0 = 最大加速度'),
    Register('Goal_Position', 42, 2, '目标位置', 'RAM', 0, 4095, 'tick', effect='motion',
             note='必须先使能位置控制，受本次动作范围限制'),
    Register('Goal_Time', 44, 2, '时间 / PWM 目标', 'RAM', 0, 65535,
             note='位置模式测试要求为 0；其他模式含义请查厂商表'),
    Register('Goal_Velocity', 46, 2, '速度限制 / 目标速度', 'RAM', -32767, 32767,
             note='单位由 Phase 决定；本界面仅执行位置模式测试'),
    Register('Torque_Limit', 48, 2, '运行输出限制', 'RAM', 0, 1000, '0.1%'),
    Register('Lock', 55, 1, 'EEPROM 写入锁', 'Feedback', note='由写入事务自动管理'),
    Register('Present_Position', 56, 2, '当前位置', 'Feedback', unit='tick'),
    Register('Present_Velocity', 58, 2, '当前速度', 'Feedback', note='由 Phase 决定单位'),
    Register('Present_Load', 60, 2, '当前输出占空比', 'Feedback', unit='0.1%',
             note='输出指示，不是直接力矩传感器'),
    Register('Present_Voltage', 62, 1, '当前电压', 'Feedback', unit='0.1 V'),
    Register('Present_Temperature', 63, 1, '当前温度', 'Feedback', unit='°C'),
    Register('Registered_Instruction', 64, 1, '异步写入标志', 'Feedback'),
    Register('Status', 65, 1, '状态位掩码', 'Feedback',
             note='bit0 电压仅警告；其他故障停止新动作并锁存'),
    Register('Moving', 66, 1, '运动状态', 'Feedback'),
    Register('Observed_Goal_Position', 67, 2, '反馈目标位置', 'Feedback', unit='tick'),
    Register('Present_Current', 69, 2, '当前电流', 'Feedback', unit='6.5 mA'),
    Register('Moving_Velocity_Threshold', 80, 1, '移动速度阈值', 'Factory'),
    Register('DTs', 81, 1, 'DTs', 'Factory', unit='ms'),
    Register('Velocity_Unit_factor', 82, 1, '速度单位系数', 'Factory'),
    Register('Hts', 83, 1, '最小速度限制', 'Factory', unit='0.732 rpm'),
    Register('Maximum_Velocity_Limit', 84, 1, '最大速度限制', 'Factory', unit='0.732 rpm'),
    Register('Maximum_Acceleration', 85, 1, '加速度限制', 'Factory'),
    Register('Acceleration_Multiplier', 86, 1, '加速度倍数', 'Factory'),
)
BY_KEY = {r.key: r for r in REGISTERS}


def schema():
    return [{**asdict(r), 'writable': r.writable} for r in REGISTERS]


def install_table(bus):
    """Patch only this bus, including adjacent single-byte addresses 24 and 25."""
    table = dict(bus.model_ctrl_table['sts3215'])
    table.update({r.key: (r.address, r.size) for r in REGISTERS})
    bus.model_ctrl_table = {**bus.model_ctrl_table, 'sts3215': table}
