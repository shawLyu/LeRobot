"""Finite single-servo steps through the native STS3215 position loop only."""

import argparse
import math
import statistics
import time
from dataclasses import replace
from pathlib import Path

from lerobot_arm_control.control.servo_feedback import read_control_feedback
from lerobot_arm_control.core.config import load_arms
from lerobot_arm_control.core.files import atomic_json
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.core.processes import interrupt
from lerobot_arm_control.diagnostics.response_metrics import DEGREES_PER_TICK
from lerobot_arm_control.diagnostics.response_metrics import native_step_metrics as metrics
from lerobot_arm_control.robot.feetech import connected_sts3215 as connected

IDENTITY_REGISTERS = ('Homing_Offset', 'Phase', 'Operating_Mode', 'Min_Position_Limit',
                      'Max_Position_Limit', 'Angular_Resolution')
TUNING_REGISTERS = ('P_Coefficient', 'I_Coefficient', 'D_Coefficient',
                    'CW_Dead_Zone', 'CCW_Dead_Zone', 'Integral_Limit',
                    'Minimum_Startup_Force')


def add_native_registers(bus):
    """Correct adjacent byte registers in the installed LeRobot 0.6.1 STS table."""
    table = dict(bus.model_ctrl_table['sts3215'])
    if table.get('Integral_Limit', (25, 1)) != (25, 1):
        raise ValueError('unexpected native integral-limit register')
    if table.get('Minimum_Startup_Force', (24, 1)) not in ((24, 1), (24, 2)):
        raise ValueError('unexpected native startup-force register')
    table['Integral_Limit'] = (25, 1)
    table['Minimum_Startup_Force'] = (24, 1)
    bus.model_ctrl_table = {**bus.model_ctrl_table, 'sts3215': table}


def identity(bus, arm, joint):
    return {'port': arm.port, 'joint': joint, 'id': arm.motors[joint], 'model': arm.model,
            'registers': {key: bus.read(key, joint, normalize=False)
                          for key in IDENTITY_REGISTERS}}


def validate_plan(zero, actual_identity, amplitudes):
    """Bind a finite, unclipped plan to the saved origin and its physical envelope."""
    if (zero.get('version') != 1 or zero['identity'] != actual_identity
            or zero['ticks_per_turn'] != 4096 or zero['direction'] not in (-1, 1)
            or type(zero['zero_raw']) is not int):
        raise ValueError('single-motor zero identity changed or is invalid')
    registers = actual_identity['registers']
    if registers['Operating_Mode'] != 0 or registers['Angular_Resolution'] != 1:
        raise ValueError('native benchmark requires position mode and native encoder scale')
    low, high = zero['allowed_degrees']
    if not -90 <= low < 0 < high <= 90:
        raise ValueError('single-motor authorization must be within +/-90 degrees')
    bounds = (max(registers['Min_Position_Limit'], math.ceil(zero['zero_raw'] +
                  low / DEGREES_PER_TICK)),
              min(registers['Max_Position_Limit'], math.floor(zero['zero_raw'] +
                  high / DEGREES_PER_TICK)))
    targets = []
    for amplitude in amplitudes:
        if not math.isfinite(amplitude) or not 1 <= amplitude <= 170:
            raise ValueError('full step amplitude must be 1..170 degrees')
        delta = round(amplitude / (2 * DEGREES_PER_TICK))
        pair = (zero['zero_raw'] - delta, zero['zero_raw'] + delta)
        if not bounds[0] + 4 <= pair[0] < pair[1] <= bounds[1] - 4:
            raise ValueError('requested step does not fit authorized travel and EEPROM limits')
        targets.append((amplitude, pair))
    if not bounds[0] <= zero['zero_raw'] <= bounds[1] or not targets:
        raise ValueError('zero outside authorized travel or empty plan')
    return bounds, targets


def zero_metrics(segment, zero_raw):
    frames = segment['frames']
    tail = [frame['position'] - zero_raw for frame in frames
            if frame['t'] >= frames[-1]['t'] - .3]
    return {'mean_error_ticks': statistics.mean(tail),
            'peak_error_ticks': max(map(abs, tail)),
            'exact_fraction': sum(error == 0 for error in tail) / len(tail)}


class NativeBench:
    def __init__(self, bus, joint, bounds, *, clock=time.monotonic, sleep=time.sleep):
        self.bus, self.joint, self.bounds = bus, joint, bounds
        self.clock, self.sleep = clock, sleep
        self.original = {key: bus.read(key, joint, normalize=False) for key in
                         (*TUNING_REGISTERS, 'Acceleration', 'Goal_Time', 'Goal_Velocity')}
        if bus.read('Lock', joint, normalize=False) != 1:
            raise ValueError('storage must remain locked throughout native tuning')
        if bus.read('Torque_Enable', joint, normalize=False) != 0:
            raise ValueError('native bench requires torque initially off')
        self.activated = False
        self.temperature, self.temperature_at = 0, -math.inf
        self.samples = []
        self.last_segment = None

    def sample(self):
        before = self.clock()
        fields = read_control_feedback(self.bus, num_retry=0)
        after = self.clock()
        frame = {'position': fields['Present_Position'][self.joint],
                 'goal': fields['Goal_Position'][self.joint],
                 'velocity': fields['Present_Velocity'][self.joint],
                 'load': fields['Present_Load'][self.joint],
                 'current_raw': fields['Present_Current'][self.joint],
                 'voltage_raw': fields['Present_Voltage'][self.joint],
                 'status': fields['Status'][self.joint],
                 'torque': fields['Torque_Enable'][self.joint],
                 'sampled_monotonic': (before + after) / 2, 'read_seconds': after - before}
        self.samples.append(frame)
        if frame['status']:
            raise RuntimeError(f"servo alarm Status={frame['status']}")
        if not self.bounds[0] <= frame['position'] <= self.bounds[1]:
            raise RuntimeError('feedback left authorized travel')
        if self.activated and frame['torque'] != 1:
            raise RuntimeError('servo torque lost during native benchmark')
        if after - self.temperature_at >= .1:
            self.temperature = self.bus.read('Present_Temperature', self.joint, normalize=False)
            self.temperature_at = self.clock()
        if self.temperature >= 50:
            raise RuntimeError('native tuning temperature reached 50 C')
        recent = [f for f in self.samples if after - f['sampled_monotonic'] <= .2]
        self.samples = recent
        if (len(recent) >= 3 and after - recent[0]['sampled_monotonic'] >= .15
                and max(f['position'] for f in recent) - min(f['position'] for f in recent) <= 2
                and all(abs(f['goal'] - f['position']) > 8 and abs(f['load']) >= 700
                        for f in recent)):
            raise RuntimeError('loaded motor stalled')
        frame['temperature_c'] = self.temperature
        return frame

    def checked_write(self, register, value):
        self.bus.write(register, self.joint, value, normalize=False)
        if self.bus.read(register, self.joint, normalize=False) != value:
            raise RuntimeError(f'{register} write/readback mismatch')

    def configure(self, gains, deadzone):
        if self.bus.read('Lock', self.joint, normalize=False) != 1:
            raise RuntimeError('storage lock changed')
        values = dict(zip(TUNING_REGISTERS[:3], gains, strict=True))
        if deadzone is not None:
            values.update(CW_Dead_Zone=deadzone, CCW_Dead_Zone=deadzone)
        if ((deadzone is not None and not 0 <= deadzone <= 16)
                or any(type(v) is not int or not 0 <= v <= 254 for v in values.values())):
            raise ValueError('invalid native coefficients')
        for register, value in values.items():
            self.checked_write(register, value)
        if self.bus.read('Lock', self.joint, normalize=False) != 1:
            raise RuntimeError('storage lock changed after tuning')

    def enable(self):
        frame = self.sample()
        # Some ST firmware activates torque on a goal write: seed with actual position first.
        self.activated = True
        self.checked_write('Goal_Position', frame['position'])
        self.checked_write('Torque_Enable', 1)

    def step(self, target, dwell, sample_hz):
        if type(target) is not int or not self.bounds[0] <= target <= self.bounds[1]:
            raise ValueError('target outside authorized travel')
        before = self.sample()
        sent = self.clock()
        self.bus.write('Goal_Position', self.joint, target, normalize=False)
        ack = self.clock()
        segment = {'start': before['position'], 'target': target,
                   'command_sent_monotonic': sent, 'command_ack_seconds': ack - sent,
                   'frames': [{**before, 't': 0.}]}
        self.last_segment = segment
        deadline = sent
        while self.clock() - sent < dwell:
            frame = self.sample()
            segment['frames'].append({**frame, 't': frame['sampled_monotonic'] - sent})
            direction = 1 if target > before['position'] else -1
            if (frame['position'] - target) * direction > max(8,
                    .15 * abs(target - before['position'])):
                raise RuntimeError('native step overshoot exceeded bounded trial allowance')
            deadline = max(deadline + 1 / sample_hz, self.clock())
            self.sleep(max(0, deadline - self.clock()))
        return segment

    def close(self):
        """Hold fresh feedback, disable torque, then restore only volatile tuning values."""
        errors = []
        if self.activated:
            try:
                if (self.bus.read('Status', self.joint, normalize=False) == 0
                        and self.bus.read('Torque_Enable', self.joint, normalize=False) == 1):
                    position = self.bus.read('Present_Position', self.joint, normalize=False)
                    self.bus.write('Goal_Position', self.joint, position, normalize=False)
            except Exception as error:
                errors.append(str(error))
            try:
                self.checked_write('Torque_Enable', 0)
            except Exception as error:
                errors.append(str(error))
        try:
            if self.bus.read('Lock', self.joint, normalize=False) != 1:
                raise RuntimeError('refusing restoration with EEPROM unlocked')
            for register, value in self.original.items():
                if self.bus.read(register, self.joint, normalize=False) != value:
                    self.checked_write(register, value)
        except Exception as error:
            errors.append(str(error))
        return errors


def capture(args):
    import json
    import signal

    zero = json.loads(args.zero.read_text())
    joint = zero['identity']['joint']
    arm = load_arms()[args.arm]
    arm = replace(arm, motors={joint: arm.motors[joint]})
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'version': 1, 'backend': 'native_servo', 'zero': zero,
              'p_i_d': [args.p, args.i, args.d], 'deadzone': args.deadzone,
              'acceleration': args.acceleration, 'integral_limit': args.integral_limit,
              'startup_force': args.startup_force,
              'started_unix_ns': time.time_ns(),
              'requested_sample_hz': args.sample_hz, 'response_target_ms': args.response_ms,
              'completed': False, 'segments': [],
              'measurement_scope': 'Quantized encoder feedback; not external absolute angle'}
    previous_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous_handlers[signum] = signal.signal(signum, interrupt)
    try:
        with connected(arm) as bus:
            add_native_registers(bus)
            actual = identity(bus, arm, joint)
            bounds, targets = validate_plan(zero, actual, args.amplitudes)
            bench = NativeBench(bus, joint, bounds)
            report['original_registers'] = bench.original
            try:
                bench.sample()
                bench.configure((args.p, args.i, args.d), args.deadzone)
                if args.startup_force is not None:
                    bench.checked_write('Minimum_Startup_Force', args.startup_force)
                if args.integral_limit is not None:
                    bench.checked_write('Integral_Limit', args.integral_limit)
                if args.acceleration is not None:
                    maximum = bus.read('Maximum_Acceleration', joint, normalize=False)
                    if not 0 <= args.acceleration <= maximum:
                        raise ValueError('acceleration exceeds native maximum')
                    bench.checked_write('Acceleration', args.acceleration)
                bench.enable()
                for amplitude, (low, high) in targets:
                    bench.step(low, args.dwell, args.sample_hz)
                    for repeat in range(args.repeats):
                        for direction, target in (('positive', high), ('negative', low)):
                            segment = bench.step(target, args.dwell, args.sample_hz)
                            segment.update(amplitude_deg=amplitude, direction=direction,
                                           repeat=repeat)
                            segment['metrics'] = metrics(segment, response_ms=args.response_ms)
                            report['segments'].append(segment)
                            measured = segment['metrics']
                            print(json.dumps({'amplitude': amplitude, 'direction': direction,
                                'repeat': repeat, 't90_s': measured['time_to_90_s'],
                                'rise_s': measured['rise_time_10_90_s'],
                                'onset_s': measured['onset_seconds'],
                                'overshoot_ticks': measured['overshoot_ticks'],
                                'tail_peak_ticks': measured['steady_peak_error_ticks'],
                                'exact_fraction': measured['exact_target_fraction']}), flush=True)
                    for approach, preposition in (('positive', low), ('negative', high)):
                        if preposition != low:
                            bench.step(preposition, args.dwell, args.sample_hz)
                        returned = bench.step(zero['zero_raw'], args.dwell, args.sample_hz)
                        returned.update(approach=approach, amplitude_deg=amplitude)
                        report.setdefault('zero_returns', []).append(returned)
                report['completed'] = True
            finally:
                if not report['completed']:
                    report['interrupted_segment'] = bench.last_segment
                    report['last_feedback'] = bench.samples
                report['restoration_errors'] = bench.close()
                report['final_identity'] = identity(bus, arm, joint)
                report['final_registers'] = {key: bus.read(key, joint, normalize=False)
                    for key in (*bench.original, 'Lock', 'Torque_Enable')}
                if report['final_identity'] != actual:
                    report['restoration_errors'].append('motor identity changed during benchmark')
                if (report['final_registers']['Lock'] != 1
                        or report['final_registers']['Torque_Enable'] != 0):
                    report['restoration_errors'].append('final lock/torque state not verified')
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error) or type(error).__name__
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        report['zero_precision_passed'] = (bool(report.get('zero_returns'))
            and all(zero_metrics(segment, zero['zero_raw'])['peak_error_ticks'] < 1
                    for segment in report.get('zero_returns', [])))
        report['performance_passed'] = (report['completed']
            and report['zero_precision_passed']
            and not report.get('error')
            and not report.get('restoration_errors')
            and all(segment['metrics']['passed'] for segment in report['segments']))
        report['finished_unix_ns'] = time.time_ns()
        atomic_json(args.output / 'result.json', report)
    print(str(args.output / 'result.json'), flush=True)
    if not report['completed'] or report.get('error') or report.get('restoration_errors'):
        raise SystemExit(1)


def capture_zero(args):
    arm = load_arms()[args.arm]
    arm = replace(arm, motors={args.joint: arm.motors[args.joint]})
    low, high = args.allowed_degrees
    if (not args.confirm_zero or not -90 <= low < 0 < high <= 90
            or args.output.exists()):
        raise ValueError('confirm a prepared zero and clear bounded travel; use a new output file')
    with connected(arm) as bus:
        if bus.read('Torque_Enable', args.joint, normalize=False) != 0:
            raise ValueError('zero capture requires torque already off')
        actual = identity(bus, arm, args.joint)
        samples = []
        for _ in range(21):
            if bus.read('Status', args.joint, normalize=False):
                raise ValueError('zero capture cannot accept a servo alarm')
            samples.append(bus.read('Present_Position', args.joint, normalize=False))
            time.sleep(.025)
        if max(samples) - min(samples) > 2:
            raise ValueError('zero pose moved during capture')
    record = {'version': 1, 'identity': actual, 'zero_raw': round(statistics.median(samples)),
              'direction': 1, 'ticks_per_turn': 4096, 'samples': samples,
              'captured_unix_ns': time.time_ns(), 'allowed_degrees': [low, high],
              'scope': 'User-defined mechanical reference; encoder positive direction; '
                       'no external sub-tick angle validation; EEPROM unchanged'}
    validate_plan(record, actual, [1])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    atomic_json(args.output, record)
    print(str(args.output))


def analyze(paths, output):
    import csv
    import gzip
    import json

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    reports = []
    for source in paths:
        path = Path(source)
        with (gzip.open(path, 'rt') if path.suffix == '.gz' else path.open()) as stream:
            reports.append((path, json.load(stream)))
    if not reports:
        raise ValueError('no native response reports')
    zero = reports[0][1]['zero']
    if any(report['zero']['identity'] != zero['identity'] or
           report['zero']['zero_raw'] != zero['zero_raw'] for _, report in reports):
        raise ValueError('cannot compare responses from different zero identities')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rows, zero_rows = [], []
    amplitudes = sorted({segment['amplitude_deg'] for _, report in reports
                         for segment in report['segments']})
    if not amplitudes:
        raise ValueError('no complete response segments to analyze')
    fig, axes = plt.subplots(math.ceil((len(amplitudes) + 1) / 2), 2,
                             figsize=(13, 3.4 * math.ceil((len(amplitudes) + 1) / 2)),
                             squeeze=False, constrained_layout=True)
    error_fig, error_axes = plt.subplots(math.ceil((len(amplitudes) + 1) / 2), 2,
        figsize=(13, 3.4 * math.ceil((len(amplitudes) + 1) / 2)),
        squeeze=False, constrained_layout=True)
    error_map = dict(zip(amplitudes, error_axes.flat))
    zero_error_axis = list(error_axes.flat)[len(amplitudes)]
    axis_map = dict(zip(amplitudes, axes.flat))
    zero_axis = list(axes.flat)[len(amplitudes)]
    for index, (path, report) in enumerate(reports):
        label = (path.parent.name if path.name == 'result.json' else
                 path.name.removesuffix('.gz').removesuffix('.json'))
        color = f'C{index % 10}'
        labelled = set()
        for segment in report['segments']:
            measured = segment['metrics']
            rows.append({'run': label, 'p': report['p_i_d'][0], 'i': report['p_i_d'][1],
                         'd': report['p_i_d'][2], 'deadzone': report['deadzone'],
                         'acceleration': report.get('acceleration'),
                         'integral_limit': report.get('integral_limit'),
                         'startup_force': report.get('startup_force'),
                         'run_completed': report['completed'],
                         'amplitude_deg': segment['amplitude_deg'],
                         'direction': segment['direction'], 'repeat': segment['repeat'],
                         **measured})
            amplitude = segment['amplitude_deg']
            denominator = segment['target'] - segment['start']
            axis_map[amplitude].plot([frame['t'] for frame in segment['frames']],
                [(frame['position'] - segment['start']) / denominator * 100
                 for frame in segment['frames']], color=color, alpha=.7,
                linestyle='-' if segment['direction'] == 'positive' else '--',
                label=label if amplitude not in labelled else None)
            tail_frames = [frame for frame in segment['frames']
                           if frame['t'] >= segment['frames'][-1]['t'] - .3]
            error_map[amplitude].plot(
                [frame['t'] - segment['frames'][-1]['t'] for frame in tail_frames],
                [segment['target'] - frame['position'] for frame in tail_frames],
                color=color, alpha=.7,
                linestyle='-' if segment['direction'] == 'positive' else '--',
                label=label if amplitude not in labelled else None)
            labelled.add(amplitude)
        for zero_index, segment in enumerate(report.get('zero_returns', [])):
            zero_rows.append({'run': label, 'amplitude_deg': segment.get('amplitude_deg'),
                              'approach': segment.get('approach', 'positive'),
                              **zero_metrics(segment, zero['zero_raw'])})
            tail_frames = [frame for frame in segment['frames']
                           if frame['t'] >= segment['frames'][-1]['t'] - .3]
            for axis in (zero_axis, zero_error_axis):
                axis.plot([frame['t'] - segment['frames'][-1]['t'] for frame in tail_frames],
                          [frame['position'] - zero['zero_raw'] for frame in tail_frames],
                          color=color, alpha=.7, label=label if zero_index == 0 else None)
    for amplitude, ax in axis_map.items():
        ax.axhline(100, color='black', linewidth=.7)
        ax.axvline(.05, color='grey', linestyle=':', linewidth=.8)
        ax.set(title=f'{amplitude:g} deg step | solid: +, dashed: -',
               xlabel='Time from command (s)', ylabel='Measured progress (%)', xlim=(0, 1.2))
        ax.grid(alpha=.2)
        ax.legend(fontsize=7)
    for amplitude, ax in error_map.items():
        ax.axhline(0, color='black', linewidth=.7)
        ax.axhspan(-1, 1, color='grey', alpha=.12)
        ax.set(title=f'{amplitude:g} deg step | settled encoder error',
               xlabel='Time to end of hold (s)', ylabel='Target minus measured (ticks)')
        ax.grid(alpha=.2)
        ax.legend(fontsize=7)
    for axis in (zero_axis, zero_error_axis):
        axis.axhline(0, color='black', linewidth=.7)
        axis.axhspan(-1, 1, color='grey', alpha=.12)
        axis.set(title='Return to user-defined encoder zero | settled window',
                 xlabel='Time to end of hold (s)', ylabel='Zero error (ticks)')
        axis.grid(alpha=.2)
        if zero_rows:
            axis.legend(fontsize=7)
    for group in (axes, error_axes):
        for ax in list(group.flat)[len(amplitudes) + 1:]:
            ax.set_visible(False)
    fig.suptitle('Single STS3215 native PID | measured encoder response | no software outer PID')
    error_fig.suptitle('Settled encoder errors | last 300 ms | shaded band: +/-1 tick')
    for name, figure in (('response', fig), ('settled-error', error_fig)):
        for extension in ('png', 'svg', 'pdf'):
            destination = output / f'{name}.{extension}'
            figure.savefig(destination, dpi=170)
            if extension == 'svg':
                destination.write_text('\n'.join(line.rstrip() for line in
                    destination.read_text().splitlines()) + '\n')
        plt.close(figure)
    with (output / 'summary.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    atomic_json(output / 'summary.json', {'responses': rows, 'zero_returns': zero_rows,
        'measurement_scope': 'Encoder feedback only. Strict <1 tick requires every settled '
        'sample to equal the integer target; '
        'this does not establish absolute mechanical accuracy.'})
    print(str(output / 'response.png'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    origin = commands.add_parser('zero',
                                 help='Read a prepared mechanical zero without motor writes')
    origin.add_argument('--arm', default='follower')
    origin.add_argument('--joint', default='shoulder_pan')
    origin.add_argument('--allowed-degrees', type=float, nargs=2, required=True)
    origin.add_argument('--confirm-zero', action='store_true')
    origin.add_argument('--output', type=Path, required=True)
    report = commands.add_parser('report', help='Plot locally downloaded native response reports')
    report.add_argument('inputs', type=Path, nargs='+')
    report.add_argument('--output', type=Path, required=True)
    trial = commands.add_parser('capture', help='Execute bounded steps with temporary native gains')
    trial.add_argument('--arm', default='follower')
    trial.add_argument('--zero', type=Path, required=True)
    trial.add_argument('--p', type=int, required=True)
    trial.add_argument('--i', type=int, required=True)
    trial.add_argument('--d', type=int, required=True)
    trial.add_argument('--deadzone', type=int, choices=range(17))
    trial.add_argument('--acceleration', type=int, choices=range(255))
    trial.add_argument('--startup-force', type=int, choices=range(255))
    trial.add_argument('--integral-limit', type=int, choices=range(255),
                       help='Native integral clamp; register units are four output counts')
    trial.add_argument('--amplitudes', type=float, nargs='+', default=[5, 15, 30, 60, 90],
                       help='Full step angle; endpoints are +/- half this value')
    trial.add_argument('--repeats', type=int, default=2)
    trial.add_argument('--dwell', type=float, default=1.2)
    trial.add_argument('--sample-hz', type=float, default=200)
    trial.add_argument('--response-ms', type=float, default=50)
    trial.add_argument('--output', type=Path,
                       default=workspace_root() / f'logs/native-pid-{time.time_ns()}')
    trial.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.command == 'zero':
        capture_zero(args)
    elif args.command == 'report':
        analyze(args.inputs, args.output)
    else:
        if (not args.execute or not 1 <= args.repeats <= 5 or not .8 <= args.dwell <= 5
                or not 50 <= args.sample_hz <= 500 or not 1 <= args.response_ms <= 1000
                or not all(0 <= value <= 254 for value in (args.p, args.i, args.d))):
            parser.error('finite native steps require --execute and valid bounded test parameters')
        capture(args)


if __name__ == '__main__':
    main()
