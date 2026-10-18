"""All motion, sampling and parameter transactions run beside the physical bus."""

import math
import time
from collections import deque
from pathlib import Path

from lerobot_arm_control.control.servo_feedback import read_control_feedback
from lerobot_arm_control.core.files import atomic_json
from lerobot_arm_control.diagnostics.response_metrics import native_step_metrics
from lerobot_arm_control.workbench.ranges import read_ranges, save_range
from lerobot_arm_control.workbench.registers import BAUDRATES, BY_KEY, REGISTERS

IDENTITY = ('Homing_Offset', 'Phase', 'Operating_Mode', 'Angular_Resolution',
            'Min_Position_Limit', 'Max_Position_Limit')


class Workbench:
    def __init__(self, bus, discovery, *, log_root, clock=time.monotonic, context=None,
                 bounds_path=None):
        self.bus, self.discovery, self.clock = bus, discovery, clock
        self.context = dict(context or {})
        self.bounds_path = Path(bounds_path) if bounds_path else None
        self.native_limits, self.bounds_info = {}, {}
        self.online = [j for j, d in discovery.items() if d['online']]
        self.log_root = Path(log_root)
        self.origin = clock()
        self.sequence = 0
        self.sample_retries = 0
        self.history = deque(maxlen=12000)
        self.frame = None
        self.enabled, self.bounds, self.last_goal = set(), {}, {}
        robot = getattr(bus, "robot_arm", None)
        self.fault = (robot.startup_report or {}).get("error") if robot else None
        self.job = self.last_result = self.move_job = None
        self.last_activity = clock()
        self.temperature_at = -math.inf
        self.temperatures, self.max_temperatures = {}, {}
        self.stall_samples = deque()
        self.audit_sequence = 0
        for joint in self.online:
            current = self.read('Present_Position', joint)
            low, high = self.position_limits(joint)
            self.native_limits[joint] = [low, high]
            # New installations start with a small, explicit envelope around the live pose.
            self.bounds[joint] = ([max(low, current - 114), min(high, current + 114)]
                                  if 0 <= low <= current <= high <= 4095 and low < high else None)
            self.max_temperatures[joint] = self.read('Max_Temperature_Limit', joint)
            self.bounds_info[joint] = {
                'source': 'default',
                'reason': ('' if self.bounds[joint] else
                           f'当前位置 {current} 不在 EEPROM 限位 {low}—{high} 内，请检查限位参数'),
            }
        self.load_bounds()
        if getattr(bus, "robot_arm", None) is not None:
            bus.robot_arm.controller = self

    def bounds_record(self, joint, bounds):
        return {'context': self.context, 'id': self.discovery[joint]['id'],
                'reference': self.identity(joint), 'bounds': bounds}

    def load_bounds(self):
        try:
            saved = read_ranges(self.bounds_path)['arms'].get(self.context.get('arm'), {})
            if not isinstance(saved, dict):
                raise ValueError('无效的机械臂范围记录')
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            for joint in self.online:
                self.bounds[joint] = None
                self.bounds_info[joint] = {
                    'source': 'invalid', 'reason': f'保存范围读取失败：{exc}'}
            return
        for joint in self.online:
            if joint not in saved:
                continue
            item = saved[joint]
            bounds = item.get('bounds') if isinstance(item, dict) else None
            low, high = self.native_limits[joint]
            if (not isinstance(bounds, list) or len(bounds) != 2
                    or any(type(v) is not int for v in bounds)
                    or not 0 <= low <= bounds[0] < bounds[1] <= high <= 4095
                    or item != self.bounds_record(joint, bounds)):
                self.bounds[joint] = None
                self.bounds_info[joint] = {'source': 'invalid', 'reason':
                    '保存范围与当前电机、零点或 EEPROM 限位不匹配，请重新检查并设置'}
            else:
                # A pose outside this interval must not silently recenter or widen it.
                self.bounds[joint] = list(bounds)
                self.bounds_info[joint] = {'source': 'saved', 'reason': ''}

    def read(self, key, joint):
        return int(self.bus.read(key, joint, normalize=False))

    def checked_write(self, key, joint, value):
        error = None
        try:
            self.bus.write(key, joint, value, normalize=False)
        except ConnectionError as exc:
            # Status level 0 omits write acknowledgements. An independent read is decisive.
            error = exc
        if BY_KEY[key].group == 'EEPROM' or key == 'Lock':
            time.sleep(.02)
        if self.read(key, joint) != value:
            raise RuntimeError(f'{key} 写入后读回不一致') from error

    def position_limits(self, joint):
        return [self.read('Min_Position_Limit', joint), self.read('Max_Position_Limit', joint)]

    def identity(self, joint):
        return {key: self.read(key, joint) for key in IDENTITY}

    def selected(self, joints):
        if (not isinstance(joints, list) or not joints or len(set(joints)) != len(joints)
                or any(j not in self.online for j in joints)):
            raise ValueError('请选择在线电机')
        return joints

    def healthy(self, joint):
        # The user explicitly requested voltage-only warnings remain nonfatal.
        status = self.read('Status', joint)
        if status & ~1:
            raise RuntimeError(f'{joint}: 舵机故障 Status={status}，禁止新动作')
        temperature = self.read('Present_Temperature', joint)
        maximum = self.read('Max_Temperature_Limit', joint)
        if maximum <= 0 or temperature >= maximum:
            raise RuntimeError(f'{joint}: 达到温度保护阈值')

    def configure_bounds(self, joint, bounds, persist=False):
        self.selected([joint])
        if (joint in self.enabled or self.job or self.move_job
                or self.read('Torque_Enable', joint) != 0):
            raise ValueError('请先停止并释放电机，再设置动作范围')
        if (not isinstance(bounds, list) or len(bounds) != 2
                or any(type(v) is not int for v in bounds)):
            raise ValueError('动作范围必须是两个整数 tick')
        low, high = self.position_limits(joint)
        current = self.read('Present_Position', joint)
        if not 0 <= low <= bounds[0] < bounds[1] <= high <= 4095:
            raise ValueError('动作范围必须处于单圈 EEPROM 位置限制内')
        if not bounds[0] <= current <= bounds[1]:
            raise ValueError('当前位置必须处于动作范围内')
        if type(persist) is not bool:
            raise ValueError('保存范围选项必须是布尔值')
        if persist:
            save_range(self.bounds_path, self.context, joint,
                       self.bounds_record(joint, list(bounds)))
        self.bounds[joint] = list(bounds)
        self.native_limits[joint] = [low, high]
        self.bounds_info[joint] = {'source': 'saved' if persist else 'session', 'reason': ''}
        return {'bounds': self.bounds[joint], 'saved': persist}

    def validate_target(self, joint, target):
        if joint not in self.enabled:
            raise ValueError(f'{joint}: 请先使能位置控制')
        if self.fault:
            raise RuntimeError('故障已锁存，请检查并点击清除故障')
        if (not self.bounds.get(joint) or type(target) is not int
                or not self.bounds[joint][0] <= target <= self.bounds[joint][1]):
            raise ValueError(f'{joint}: 目标超出本次动作范围 {self.bounds.get(joint)}')

    def enable(self, joints):
        self.selected(joints)
        if self.fault or self.job or self.move_job:
            raise ValueError('请先结束动作或处理故障')
        positions = {}
        for joint in joints:
            self.healthy(joint)
            if (self.read('Operating_Mode', joint) != 0
                    or self.read('Angular_Resolution', joint) != 1
                    or self.read('Goal_Time', joint) != 0 or self.read('Lock', joint) != 1):
                raise ValueError('位置控制要求模式 0、分辨率 1、Goal_Time=0、EEPROM 已锁定')
            position = self.read('Present_Position', joint)
            if not self.bounds.get(joint):
                raise ValueError('参考参数已变化，请先重新设置动作范围')
            low, high = self.bounds[joint]
            if not low <= position <= high:
                raise ValueError('当前位置不在动作范围；请重新设置范围')
            positions[joint] = position
        # Seed the current pose: some STS firmware enables torque on a goal write.
        for joint, position in positions.items():
            self.enabled.add(joint)
            self.checked_write('Goal_Position', joint, position)
            self.checked_write('Torque_Enable', joint, 1)
            self.last_goal[joint] = float(position)
        self.last_activity = self.clock()
        return {'enabled': sorted(self.enabled)}

    def hold(self, reason='用户停止'):
        self.move_job = None
        errors = []
        for joint in list(self.enabled):
            try:
                if not self.read('Status', joint) & ~1 and self.read('Torque_Enable', joint) == 1:
                    actual = self.read('Present_Position', joint)
                    self.checked_write('Goal_Position', joint, actual)
                    self.last_goal[joint] = float(actual)
            except Exception as exc:
                errors.append(f'{joint}: {exc}')
        if self.job:
            self.finish_job(reason)
        return {'held': sorted(self.enabled), 'errors': errors, 'reason': reason}

    def release(self, joints):
        self.selected(joints)
        result = self.hold('释放扭矩')
        for joint in joints:
            self.checked_write('Torque_Enable', joint, 0)
            self.enabled.discard(joint)
        return {**result, 'enabled': sorted(self.enabled)}

    def clear_fault(self):
        for joint in self.online:
            self.healthy(joint)
            if self.read('Lock', joint) != 1:
                if self.read('Torque_Enable', joint) != 0:
                    raise ValueError('请先释放扭矩，再恢复 EEPROM 写入锁')
                self.checked_write('Lock', joint, 1)
        robot = getattr(self.bus, "robot_arm", None)
        if robot and robot.startup_report and robot.startup_report.get("error"):
            references = {j: self.identity(j) for j in self.online}
            robot.check_parameters()
            # Restored reference limits may change the valid software envelope.
            for joint in self.online:
                self.native_limits[joint] = self.position_limits(joint)
                if references[joint] != self.identity(joint):
                    self.bounds[joint] = None
                    self.bounds_info[joint] = {"source": "invalid",
                                               "reason": "保存参考参数已恢复，请重新设置动作范围"}
        self.fault = None
        self.stall_samples.clear()
        return {'fault': None}

    def parameters(self, joint):
        self.selected([joint])
        if self.job or self.move_job:
            raise ValueError('运动采样期间不能读取完整参数表，请先停止')
        values, errors = {}, {}
        for register in REGISTERS:
            try:
                values[register.key] = self.read(register.key, joint)
            except (ConnectionError, RuntimeError, ValueError) as exc:
                errors[register.key] = str(exc)
        return {'joint': joint, 'values': values, 'errors': errors}

    def write_settings(self, joint, changes, expected, persist=False):
        self.selected([joint])
        if (not isinstance(changes, dict) or not changes or len(changes) > len(REGISTERS)
                or type(persist) is not bool or not isinstance(expected, dict)
                or set(expected) != set(changes)):
            raise ValueError('需要修改项及对应原值')
        for key, value in changes.items():
            if key not in BY_KEY:
                raise ValueError(f'未知参数 {key}')
            BY_KEY[key].check(value)
        if 'Goal_Position' in changes:
            if len(changes) != 1 or persist:
                raise ValueError('目标位置请单独作为运动指令发送')
            return self.move({joint: changes['Goal_Position']}, 120)
        if 'Torque_Enable' in changes:
            if len(changes) != 1 or persist:
                raise ValueError('扭矩请单独操作')
            value = changes['Torque_Enable']
            if value == 0:
                return self.release([joint])
            if value == 1:
                return self.enable([joint])
            self.release([joint])
            self.checked_write('Torque_Enable', joint, 2)
            return {'damping': joint}
        if self.job or self.move_job or self.enabled or any(
                self.read('Torque_Enable', j) != 0 for j in self.online):
            raise ValueError('修改参数前请释放所有在线电机的扭矩')
        self.healthy(joint)
        if self.read('Lock', joint) != 1:
            raise ValueError('检测到 EEPROM 未锁定，请先重新连接并检查设备')
        if {'Goal_Time', 'Goal_Velocity'} & changes.keys() and self.read('Operating_Mode', joint):
            raise ValueError('此模式下该参数可直接驱动电机；请先切换到位置模式 0 再编辑')
        before = {key: self.read(key, joint) for key in changes}
        if before != expected:
            raise ValueError('参数已变化，请刷新后再修改')
        transport = [key for key in changes if BY_KEY[key].effect == 'transport']
        if transport and (len(changes) != 1 or len(self.online) != 1):
            raise ValueError('修改 ID 或波特率时总线必须仅有一只在线舵机，且一次只改一个参数')
        if 'ID' in changes and any(d['id'] == changes['ID'] for j, d in
                                  self.discovery.items() if j != joint):
            raise ValueError('新 ID 与其他通道的 ID 重复')
        if ('ID' in changes and changes['ID'] != before['ID']
                and not self.bus.id_available(changes['ID'])):
            raise ValueError('新 ID 已被总线上的其他设备占用')
        if persist and any(BY_KEY[key].group != 'EEPROM' for key in changes):
            raise ValueError('RAM 参数不能保存到 EEPROM')
        for low_key, high_key in (('Min_Position_Limit', 'Max_Position_Limit'),
                                  ('Min_Voltage_Limit', 'Max_Voltage_Limit')):
            if {low_key, high_key} & changes.keys():
                low = changes.get(low_key, self.read(low_key, joint))
                high = changes.get(high_key, self.read(high_key, joint))
                if not low < high and not (low == high == 0 and 'Position' in low_key):
                    raise ValueError(f'{low_key} 必须小于 {high_key}')
                if (low_key == 'Min_Position_Limit' and low < high
                        and not {'Homing_Offset', 'Phase', 'Angular_Resolution',
                                 'Operating_Mode'} & changes.keys()
                        and self.read('Operating_Mode', joint) == 0
                        and not low <= self.read('Present_Position', joint) <= high):
                    raise ValueError('新 EEPROM 位置限位必须包含当前位置，请先手动移入拟定范围')
        audit = {'joint': joint, 'id': self.discovery[joint]['id'], 'before': before,
                 'requested': changes, 'persist': persist, 'unix_ns': time.time_ns(),
                 'device_context': self.context.copy(),
                 'reference': self.identity(joint),
                 'completed': False, 'rollback_errors': []}
        if any(BY_KEY[key].effect == 'reference' for key in changes):
            self.bounds[joint] = None
            self.bounds_info[joint] = {'source': 'invalid', 'reason':
                '参考参数已修改，请重新检查并设置动作范围'}
        try:
            if persist:
                self.checked_write('Lock', joint, 0)
            for key, value in changes.items():
                if value == before[key]:
                    continue
                if key in transport:
                    self.write_transport(joint, key, value)
                else:
                    self.checked_write(key, joint, value)
                if self.read('Torque_Enable', joint) != 0:
                    raise RuntimeError('参数写入意外使能扭矩，已中止并释放')
            audit['after'] = {key: self.read(key, joint) for key in changes}
            audit['completed'] = True
        except BaseException as exc:
            audit['error'] = str(exc)
            if not transport:
                for key, value in before.items():
                    try:
                        self.checked_write(key, joint, value)
                    except Exception as rollback:
                        audit['rollback_errors'].append(str(rollback))
            raise
        finally:
            torque_error = None
            try:
                if self.read('Torque_Enable', joint) != 0:
                    self.checked_write('Torque_Enable', joint, 0)
            except Exception as exc:
                torque_error = exc
                audit['completed'] = False
                audit['torque_error'] = str(exc)
                self.fault = f'配置后无法确认扭矩关闭：{exc}'
            try:
                self.checked_write('Lock', joint, 1)
                audit['lock_verified'] = True
            except Exception as exc:
                audit['lock_verified'] = False
                audit['completed'] = False
                self.fault = f'EEPROM 写入锁无法确认：{exc}'
                raise
            finally:
                self.native_limits[joint] = self.position_limits(joint)
                if audit['rollback_errors']:
                    self.fault = '参数恢复失败，请检查读回值后再使能'
                self.audit_sequence += 1
                try:
                    self.log_root.mkdir(parents=True, exist_ok=True)
                    atomic_json(self.log_root / f'parameters-{self.audit_sequence}.json', audit)
                except OSError as exc:
                    audit['log_error'] = str(exc)
            if torque_error:
                raise RuntimeError(self.fault) from torque_error
        self.max_temperatures[joint] = self.read('Max_Temperature_Limit', joint)
        if 'Baud_Rate' in changes:
            self.context['baudrate'] = BAUDRATES[changes['Baud_Rate']]
        return {**audit, 'ids': {j: d['id'] for j, d in self.discovery.items()},
                'bounds': self.bounds[joint],
                'baudrate': BAUDRATES[changes['Baud_Rate']] if 'Baud_Rate' in changes else None}

    def write_transport(self, joint, key, value):
        error = None
        try:
            self.bus.write(key, joint, value, normalize=False)
        except ConnectionError as exc:
            error = exc
        if key == 'ID':
            self.bus.set_id(joint, value)
            self.discovery[joint]['id'] = value
        else:
            self.bus.set_baudrate(BAUDRATES[value])
        time.sleep(.05)
        if self.read(key, joint) != value:
            raise RuntimeError('通信参数未读回；请断电后使用新 ID / 波特率重新扫描') from error

    def move(self, targets, speed_deg_s):
        if self.job:
            raise ValueError('响应测试正在进行，请先停止')
        self.selected(list(targets))
        if (type(speed_deg_s) not in (int, float) or not math.isfinite(speed_deg_s)
                or not 1 <= speed_deg_s <= 600):
            raise ValueError('移动速度必须在 1..600 °/s')
        for joint, target in targets.items():
            self.validate_target(joint, target)
            self.healthy(joint)
        self.move_job = {'targets': dict(targets), 'ticks_s': speed_deg_s * 4096 / 360,
                         'at': self.clock()}
        return {'targets': targets, 'phase': 'moving'}

    def start_step(self, joints, amplitude_deg, repeats, dwell):
        self.selected(joints)
        if self.job or self.move_job:
            raise ValueError('请先停止当前动作')
        if (type(amplitude_deg) not in (int, float) or not math.isfinite(amplitude_deg)
                or not 1 <= amplitude_deg <= 170 or type(repeats) is not int
                or not 1 <= repeats <= 5 or type(dwell) not in (int, float)
                or not math.isfinite(dwell) or not .8 <= dwell <= 5):
            raise ValueError('阶跃幅值 1..170°、重复 1..5 次、保持 0.8..5 秒')
        delta = round(amplitude_deg * 4096 / 720)
        plan, native = [], {}
        for joint in joints:
            self.healthy(joint)
            center = self.read('Present_Position', joint)
            for target in (center - delta, center + delta):
                self.validate_target(joint, target)
            native[joint] = {key: self.read(key, joint) for key in (
                'P_Coefficient', 'I_Coefficient', 'D_Coefficient', 'CW_Dead_Zone',
                'CCW_Dead_Zone', 'Integral_Limit', 'Minimum_Startup_Force',
                'Acceleration', 'Goal_Time', 'Goal_Velocity')}
            plan.append({'joint': joint, 'target': center - delta, 'kind': 'preposition'})
            for repeat in range(repeats):
                for direction, target in (('positive', center + delta),
                                          ('negative', center - delta)):
                    plan.append({'joint': joint, 'target': target, 'kind': 'step',
                                 'direction': direction, 'repeat': repeat})
            plan.append({'joint': joint, 'target': center, 'kind': 'return'})
        self.last_result = None
        self.job = {'id': str(time.time_ns()), 'plan': plan, 'index': 0, 'dwell': dwell,
                    'amplitude_deg': amplitude_deg, 'segments': [], 'active': None,
                    'device_context': self.context.copy(),
                'parameter_check': (self.bus.robot_arm.startup_report
                                    if getattr(self.bus, 'robot_arm', None) else None),
                    'motor_identities': {j: {'id': self.discovery[j]['id'], **self.identity(j)}
                                         for j in joints},
                    'native_parameters': native, 'started_unix_ns': time.time_ns()}
        return {'job_id': self.job['id'], 'segments': len(plan)}

    def finish_job(self, error=None):
        job, self.job = self.job, None
        if not job:
            return
        result = {key: value for key, value in job.items() if key not in ('plan', 'active')}
        result.update(completed=error is None, error=error, finished_unix_ns=time.time_ns())
        if job['active'] and error:
            result['interrupted_segment'] = job['active']
        self.last_result = result
        try:
            self.log_root.mkdir(parents=True, exist_ok=True)
            atomic_json(self.log_root / f"response-{job['id']}.json", result)
        except OSError as exc:
            result['log_error'] = str(exc)

    def advance(self, now, frame):
        if self.move_job:
            job = self.move_job
            maximum = job['ticks_s'] * min(.1, max(0, now - job['at']))
            goals = {}
            for joint, target in job['targets'].items():
                previous = self.last_goal[joint]
                following = previous + max(-maximum, min(maximum, target - previous))
                self.last_goal[joint] = following
                goals[joint] = round(following)
            if goals:
                self.bus.sync_write('Goal_Position', goals, normalize=False)
            job['at'] = now
            if all(round(self.last_goal[j]) == v for j, v in job['targets'].items()):
                self.move_job = None
        if not self.job:
            return
        job = self.job
        segment = job['active']
        if segment:
            joint = segment['joint']
            position = frame['motors'][joint]['position']
            segment['frames'].append({'t': now - segment['sent'], 'position': position})
            direction = 1 if segment['target'] > segment['start'] else -1
            if ((position - segment['target']) * direction > max(8,
                    .15 * abs(segment['target'] - segment['start']))):
                self.fault = f'{joint}: 阶跃超调超过测试允许值'
                self.hold(self.fault)
                return
            if now - segment['sent'] >= job['dwell']:
                segment['metrics'] = native_step_metrics(segment)
                job['segments'].append(segment)
                job['active'] = None
                job['index'] += 1
            else:
                return
        if job['index'] >= len(job['plan']):
            self.finish_job()
            return
        entry = job['plan'][job['index']]
        joint, target = entry['joint'], entry['target']
        self.validate_target(joint, target)
        start = frame['motors'][joint]['position']
        sent = self.clock()
        self.bus.sync_write('Goal_Position', {joint: target}, normalize=False)
        ack = self.clock() - sent
        self.last_goal[joint] = float(target)
        job['active'] = {**entry, 'start': start, 'sent': sent, 'target': target,
                         'command_ack_seconds': ack,
                         'frames': [{'t': 0., 'position': start}]}

    def tick(self):
        if not self.online:
            return self.state()
        before = self.clock()
        fields = read_control_feedback(self.bus, num_retry=0)
        after = self.clock()
        now = (before + after) / 2
        if after - self.temperature_at >= .25:
            self.temperatures = {j: self.read('Present_Temperature', j) for j in self.online}
            self.temperature_at = after
        motors = {j: {'position': fields['Present_Position'][j],
                      'goal': fields['Goal_Position'][j],
                      'velocity': fields['Present_Velocity'][j],
                      'load': fields['Present_Load'][j],
                      'current_raw': fields['Present_Current'][j],
                      'voltage': fields['Present_Voltage'][j] / 10,
                      'temperature': self.temperatures.get(j),
                      'status': fields['Status'][j], 'torque': fields['Torque_Enable'][j]}
                  for j in self.online}
        self.sequence += 1
        frame = {'seq': self.sequence, 't': now - self.origin, 'read_seconds': after - before,
                 'motors': motors}
        self.frame = frame
        self.history.append(frame)
        for joint, values in motors.items():
            issue = None
            if values['status'] & ~1:
                issue = f"{joint}: 舵机故障 Status={values['status']}"
            elif joint in self.enabled and values['torque'] != 1:
                issue = f'{joint}: 扭矩意外关闭'
            elif joint in self.enabled and self.bounds.get(joint) and not (
                    self.bounds[joint][0] <= values['position'] <= self.bounds[joint][1]):
                issue = f'{joint}: 反馈超出动作范围'
            elif joint in self.enabled and values['temperature'] >= self.max_temperatures[joint]:
                issue = f'{joint}: 达到温度保护阈值'
            if issue and self.fault is None:
                self.fault = issue
                self.hold(issue)
                self.enabled.clear()
        self.stall_samples.append(frame)
        while self.stall_samples and now - self.origin - self.stall_samples[0]['t'] > .3:
            self.stall_samples.popleft()
        if self.enabled and len(self.stall_samples) >= 3 and (
                frame['t'] - self.stall_samples[0]['t'] >= .25):
            for joint in self.enabled:
                samples = [f['motors'][joint] for f in self.stall_samples]
                if (max(s['position'] for s in samples) - min(s['position'] for s in samples) <= 2
                        and all(abs(s['goal'] - s['position']) > 32 and abs(s['load']) >= 700
                                for s in samples)):
                    self.fault = f'{joint}: 高输出受阻，已保持'
                    self.hold(self.fault)
                    self.enabled.clear()
                    break
        if self.enabled and self.clock() - self.last_activity > 1.5:
            self.hold('界面心跳超时，已保持；需重新使能')
            self.enabled.clear()
        if not self.fault:
            self.advance(now, frame)
        return self.state()

    def state(self):
        return {'frame': self.frame, 'discovery': self.discovery, 'enabled': sorted(self.enabled),
                'device_context': self.context.copy(),
                'parameter_check': (self.bus.robot_arm.startup_report
                                    if getattr(self.bus, 'robot_arm', None) else None),
                'bounds': self.bounds, 'bounds_info': self.bounds_info,
                'native_limits': self.native_limits, 'fault': self.fault,
                'sample_retries': self.sample_retries,
                'motion': 'response' if self.job else 'moving' if self.move_job else 'idle',
                'job': ({'id': self.job['id'], 'index': self.job['index'],
                         'total': len(self.job['plan'])} if self.job else None),
                'result_id': self.last_result['id'] if self.last_result else None}

    def command(self, operation, payload):
        self.last_activity = self.clock()
        if operation == 'heartbeat':
            return {}
        if operation == 'parameters':
            return self.parameters(payload['joint'])
        if operation == 'write':
            return self.write_settings(payload['joint'], payload['changes'],
                                       payload['expected'], payload.get('persist', False))
        if operation == 'bounds':
            return self.configure_bounds(payload['joint'], payload['bounds'],
                                         payload.get('persist', False))
        if operation == 'enable':
            return self.enable(payload['joints'])
        if operation == 'hold':
            return self.hold()
        if operation == 'release':
            return self.release(payload['joints'])
        if operation == 'clear_fault':
            return self.clear_fault()
        if operation == 'move':
            return self.move(payload['targets'], payload.get('speed_deg_s', 60))
        if operation == 'step':
            return self.start_step(payload['joints'], payload['amplitude_deg'],
                                   payload.get('repeats', 1), payload.get('dwell', 1.2))
        if operation == 'export':
            if self.job or self.move_job:
                raise ValueError('请等待运动结束后导出完整采样')
            return {'version': 1, 'state': self.state(), 'frames': list(self.history),
                    'response': self.last_result}
        if operation == 'result':
            return self.last_result
        raise ValueError(f'未知操作 {operation}')
