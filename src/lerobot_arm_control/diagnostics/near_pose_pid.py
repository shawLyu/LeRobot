"""Finite native PID search around a held pose, without dropping supporting joints."""

import argparse
import json
import math
import signal
import statistics
import time
from pathlib import Path

from lerobot_arm_control.core.config import load_arms
from lerobot_arm_control.core.files import atomic_json
from lerobot_arm_control.core.processes import interrupt
from lerobot_arm_control.diagnostics.native_pid import TUNING_REGISTERS
from lerobot_arm_control.diagnostics.response_metrics import DEGREES_PER_TICK, native_step_metrics
from lerobot_arm_control.workbench.controller import Workbench
from lerobot_arm_control.workbench.hardware import open_hardware
from lerobot_arm_control.workbench.registers import BY_KEY


class CandidateRejected(RuntimeError):
    """A measured response was too oscillatory; restore the known settings."""


def summarize(segments):
    steps = [s for s in segments if s['kind'] == 'step']
    if not steps:
        raise ValueError('both approach directions must be measured')
    metrics = [s['metrics'] for s in steps]
    tails = [[f['position'] for f in s['frames'] if f['t'] >= s['frames'][-1]['t'] - .4]
             for s in steps]
    result = {
        'steady_peak_ticks': max(m['steady_peak_error_ticks'] for m in metrics),
        'steady_rms_ticks': statistics.mean(m['steady_rms_error_ticks'] for m in metrics),
        'jitter_peak_to_peak_ticks': max(max(t) - min(t) for t in tails),
        'jitter_std_ticks': max(statistics.pstdev(t) for t in tails),
        'overshoot_ticks': max(m['overshoot_ticks'] for m in metrics),
        't90_ms': (max(m['time_to_90_s'] * 1000 for m in metrics)
                   if all(m['time_to_90_s'] is not None for m in metrics) else None),
        'max_sample_gap_ms': max(m['max_sample_gap_s'] * 1000 for m in metrics),
    }
    result['acceptable'] = (result['steady_peak_ticks'] <= 2
                            and result['jitter_peak_to_peak_ticks'] <= 2
                            and result['overshoot_ticks'] <= 3
                            and result['t90_ms'] is not None and result['t90_ms'] <= 200
                            and result['max_sample_gap_ms'] <= 25)
    # A low signed mean must not conceal oscillation; speed is a secondary objective.
    result['score'] = (8 * max(0, result['jitter_peak_to_peak_ticks'] - 2)
                       + 5 * max(0, result['steady_peak_ticks'] - 2)
                       + 8 * max(0, result['overshoot_ticks'] - 3)
                       + 2 * result['steady_rms_ticks'] + result['jitter_std_ticks']
                       + .4 * result['overshoot_ticks']
                       + (result['t90_ms'] / 100 if result['t90_ms'] is not None else 100)
                       + 10 * max(0, result['max_sample_gap_ms'] - 25))
    return result


def select_finalists(trials, count=3):
    """Compare distinct profiles using their worst observed result, not a lucky repeat."""
    groups = {}
    for trial in trials:
        key = tuple(sorted(trial['settings'].items()))
        groups.setdefault(key, []).append(trial)
    eligible = [max(group, key=lambda t: t['summary']['score']) for group in groups.values()
                if all(t['completed'] for t in group)]
    return sorted(eligible, key=lambda t: t['summary']['score'])[:count]


def profile(original, p, i, d, *, force=4, deadzone=1, limit=8):
    return {**original, 'P_Coefficient': p, 'I_Coefficient': i, 'D_Coefficient': d,
            'Minimum_Startup_Force': force, 'CW_Dead_Zone': deadzone,
            'CCW_Dead_Zone': deadzone, 'Integral_Limit': limit}


class PoseSweep:
    def __init__(self, controller, radius_deg=5, anchor=None):
        self.control = controller
        self.sample_retries = 0
        if not math.isfinite(radius_deg) or not 2 <= radius_deg <= 5:
            raise ValueError('pose radius must be 2..5 degrees')
        self.original, self.identities, self.centers = {}, {}, {}
        if anchor and set(anchor['centers']) != set(controller.online):
            raise ValueError('anchor must cover every online joint')
        for joint in controller.online:
            controller.healthy(joint)
            if (controller.read('Torque_Enable', joint) != 1
                    or controller.read('Lock', joint) != 1
                    or controller.read('Operating_Mode', joint) != 0
                    or controller.read('Angular_Resolution', joint) != 1
                    or controller.read('Goal_Time', joint) != 0):
                raise ValueError('all online joints must already hold position with storage locked')
            actual = controller.read('Present_Position', joint)
            center = anchor['centers'][joint] if anchor else actual
            if anchor and controller.identity(joint) != anchor['identities'][joint]:
                raise ValueError('anchor mechanical reference changed')
            low, high = controller.position_limits(joint)
            radius = round(radius_deg / DEGREES_PER_TICK)
            bounds = [max(low + 4, center - radius), min(high - 4, center + radius)]
            if not (bounds[0] + 16 <= center <= bounds[1] - 16
                    and bounds[0] <= actual <= bounds[1]):
                raise ValueError(f'{joint}: insufficient clearance around held pose')
            if abs(controller.read('Goal_Position', joint) - actual) > 20:
                raise ValueError(f'{joint}: existing target is not settled near the held pose')
            self.centers[joint] = center
            controller.bounds[joint] = bounds
            self.identities[joint] = controller.identity(joint)
            self.original[joint] = {k: controller.read(k, joint) for k in TUNING_REGISTERS}
        controller.enabled = set(controller.online)
        controller.last_goal = {j: float(controller.read('Goal_Position', j))
                                for j in controller.online}

    def sample(self):
        c = self.control
        c.last_activity = c.clock()
        started = c.clock()
        try:
            state = c.tick()
        except ConnectionError:
            if c.clock() - started > .05:
                raise
            self.sample_retries += 1
            state = c.tick()
        if c.fault:
            raise RuntimeError(c.fault)
        if any(m['temperature'] >= 50 for m in state['frame']['motors'].values()):
            c.hold('tuning temperature limit 50 C')
            raise RuntimeError('tuning temperature limit 50 C')
        return state['frame']

    def configure(self, joint, settings):
        c = self.control
        if set(settings) != set(TUNING_REGISTERS):
            raise ValueError('only the seven native tuning registers are allowed')
        for key, value in settings.items():
            BY_KEY[key].check(value)
        if settings['P_Coefficient'] < 8:
            raise ValueError('supporting joint P must remain at least 8')
        if c.read('Lock', joint) != 1 or c.identity(joint) != self.identities[joint]:
            raise RuntimeError('storage lock or mechanical reference changed')
        self.sample()
        if {k: c.read(k, joint) for k in settings} == settings:
            return
        # No torque release or goal write: supporting joints retain their pose.
        # Suppress integral accumulation while replacing the other coefficients.
        order = {'I_Coefficient': 0, **{k: settings[k] for k in TUNING_REGISTERS
                                      if k != 'I_Coefficient'}}
        for key, value in order.items():
            if c.read(key, joint) != value:
                c.checked_write(key, joint, value)
            self.sample()
        c.checked_write('I_Coefficient', joint, settings['I_Coefficient'])
        if c.read('Lock', joint) != 1:
            raise RuntimeError('storage lock changed during tuning')
        if {k: c.read(k, joint) for k in settings} != settings:
            raise RuntimeError('candidate readback mismatch')

    def segment(self, joint, target, kind, dwell=1.2):
        c = self.control
        frame = self.sample()
        c.validate_target(joint, target)
        sent = c.clock()
        c.bus.sync_write('Goal_Position', {joint: target}, normalize=False)
        ack = c.clock() - sent
        c.last_goal[joint] = float(target)
        segment = {'joint': joint, 'target': target, 'start': frame['motors'][joint]['position'],
                   'kind': kind, 'command_sent_monotonic': sent,
                   'command_ack_seconds': ack, 'frames': []}
        self.last_segment = segment
        if kind == 'step' and abs(target - segment['start']) < 4:
            raise CandidateRejected('insufficient measured step excursion')
        segment['frames'].append({'t': 0., **frame['motors'][joint]})
        deadline = sent
        while c.clock() - sent < dwell:
            frame = self.sample()
            motor = frame['motors'][joint]
            segment['frames'].append({'t': c.origin + frame['t'] - sent, **motor})
            direction = 1 if target > segment['start'] else -1
            if kind == 'step' and (motor['position'] - target) * direction > 8:
                raise CandidateRejected('overshoot above 8 ticks')
            tail = [f['position'] for f in segment['frames'] if f['t'] > .6]
            if len(tail) > 20 and max(tail) - min(tail) > 10:
                raise CandidateRejected('persistent jitter above 10 ticks')
            deadline = max(deadline + .01, c.clock())
            time.sleep(max(0, deadline - c.clock()))
        if target != segment['start']:
            segment['metrics'] = native_step_metrics(segment)
        return segment

    def trial(self, joint, settings, *, amplitude=3., repeats=1):
        c = self.control
        center = self.centers[joint]
        delta = round(amplitude / (2 * DEGREES_PER_TICK))
        for target in (center - delta, center + delta):
            c.validate_target(joint, target)
        self.configure(joint, settings)
        result = {'settings': settings, 'amplitude_deg': amplitude, 'segments': [],
                  'completed': False}
        try:
            result['segments'].append(self.segment(joint, center - delta, 'preposition'))
            for _ in range(repeats):
                for target in (center + delta, center - delta):
                    result['segments'].append(self.segment(joint, target, 'step'))
            result['segments'].append(self.segment(joint, center, 'return'))
            result.update(completed=True, summary=summarize(result['segments']))
        except CandidateRejected as exc:
            result['error'] = str(exc)
            result['interrupted_segment'] = self.last_segment
            # This is a response rejection, never a device alarm reset.
            self.configure(joint, self.original[joint])
            c.healthy(joint)
            actual = c.read('Present_Position', joint)
            c.validate_target(joint, actual)
            c.checked_write('Goal_Position', joint, actual)
            c.last_goal[joint] = float(actual)
        return result


def run(args):
    arm = load_arms()[args.arm]
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'version': 1, 'started_unix_ns': time.time_ns(),
              'completed': False, 'joints': {}, 'scope':
              'Native encoder response near current pose and load; not global PID optimum'}
    with open_hardware(arm, list(arm.motors.values()), arm.baudrate) as (bus, discovery):
        c = Workbench(bus, discovery, log_root=args.output / 'workbench')
        anchor = json.loads(args.anchor.read_text()) if args.anchor else None
        sweep = PoseSweep(c, anchor=anchor)
        report.update(centers=sweep.centers, identities=sweep.identities,
                      original=sweep.original, discovery=discovery)
        selected = dict(sweep.original)
        atomic_json(args.output / 'result.json', report)
        try:
            for joint in args.joints or c.online:
                c.selected([joint])
                trials = []
                report['joints'][joint] = {'trials': trials}
                original = sweep.original[joint]

                def measure(settings, amplitude=3., repeats=1, label='search'):
                    result = sweep.trial(joint, settings, amplitude=amplitude, repeats=repeats)
                    result['label'] = label
                    trials.append(result)
                    atomic_json(args.output / 'result.json', report)
                    print(json.dumps({'joint': joint, 'trial': len(trials), 'label': label,
                                      'pid': [settings[k] for k in TUNING_REGISTERS[:3]],
                                      'summary': result.get('summary'),
                                      'error': result.get('error')}), flush=True)
                    return result

                measure(original, label='baseline')
                measure(original, amplitude=5., repeats=2, label='baseline-validation')
                if args.profiles:
                    requested = json.loads(args.profiles.read_text())[joint]
                    for settings in requested:
                        measure({**original, **settings}, amplitude=args.search_amplitude)
                else:
                    candidates = [(16, 0, 48), (24, 0, 64), (32, 0, 96), (48, 0, 128),
                                  (64, 0, 128), (48, 0, 192), (32, 0, 160)]
                    for p, i, d in candidates:
                        measure(profile(original, p, i, d))
                    good = [r for r in trials if r['completed']]
                    if not good:
                        raise RuntimeError(f'{joint}: no stable candidate')
                    best = min(good, key=lambda r: r['summary']['score'])['settings']
                    for integral in (1, 2, 4, 8):
                        measure({**best, 'I_Coefficient': integral, 'Integral_Limit': 8})
                    for p, i, d in ((32, 4, 64), (48, 4, 96), (48, 4, 128),
                                    (64, 4, 128)):
                        measure(profile(original, p, i, d, force=8))
                    best = min((r for r in trials if r['completed']),
                               key=lambda r: r['summary']['score'])['settings']
                    for force, dead in ((0, 1), (4, 0), (0, 0)):
                        measure({**best, 'Minimum_Startup_Force': force,
                                 'CW_Dead_Zone': dead, 'CCW_Dead_Zone': dead})
                finalists = select_finalists(trials)
                validations = []
                for finalist in finalists:
                    validation = measure(finalist['settings'], amplitude=5., repeats=3,
                                         label='validation')
                    if validation['completed']:
                        try:
                            held = sweep.segment(joint, sweep.centers[joint], 'hold', dwell=6.)
                            validation['hold_validation'] = held
                            tail = [f['position'] for f in held['frames'] if f['t'] >= 3.]
                            validation['hold_peak_error_ticks'] = max(
                                abs(v - sweep.centers[joint]) for v in tail)
                            validation['hold_jitter_ticks'] = max(tail) - min(tail)
                            validation['summary']['score'] += (
                                8 * max(0, validation['hold_jitter_ticks'] - 2)
                                + 5 * max(0, validation['hold_peak_error_ticks'] - 2))
                            validation['summary']['acceptable'] &= (
                                validation['hold_jitter_ticks'] <= 2
                                and validation['hold_peak_error_ticks'] <= 2)
                            validations.append(validation)
                        except CandidateRejected as exc:
                            validation['hold_error'] = str(exc)
                            sweep.configure(joint, original)
                if not validations:
                    raise RuntimeError(f'{joint}: finalists failed validation')
                winner = min(validations, key=lambda r: r['summary']['score'])
                selected[joint] = winner['settings']
                sweep.configure(joint, winner['settings'])
                report['joints'][joint]['selected'] = winner
                atomic_json(args.output / 'result.json', report)
            report['completed'] = True
        except BaseException as exc:
            report['error'] = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            errors = []
            for joint, settings in selected.items():
                try:
                    sweep.configure(joint, settings)
                except Exception as exc:
                    errors.append(f'{joint}: {exc}')
            report['finished_unix_ns'] = time.time_ns()
            report['sample_retries'] = sweep.sample_retries
            report['restoration_errors'] = errors
            report['hold'] = c.hold('finite near-pose sweep finished')
            report['final'] = {j: {k: c.read(k, j) for k in (*TUNING_REGISTERS,
                              'Lock', 'Torque_Enable', 'Present_Position', 'Status')}
                               for j in c.online}
            atomic_json(args.output / 'result.json', report)



def report_curves(paths, output):
    """Render baseline/final traces; no device or optional hardware imports are opened."""
    import csv

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    measured = {}
    for path in paths:
        record = json.loads(path.read_text())
        for joint, result in record['joints'].items():
            if 'selected' in result:
                if joint in measured:
                    measured[joint]['selected'] = result['selected']
                else:
                    measured[joint] = dict(result)
    if not measured:
        raise ValueError('no completed joint selections')
    figure, axes = plt.subplots(len(measured), 2, figsize=(12, 3 * len(measured)),
                                squeeze=False, constrained_layout=True)
    rows = []
    for axis, (joint, result) in zip(axes, measured.items(), strict=True):
        baseline = next(t for t in result['trials'] if t['label'] == 'baseline-validation')
        steady_extent = 3
        for label, trial, color in [('original', baseline, '#ac533a'),
                                    ('selected', result['selected'], '#167d9a')]:
            steps = [s for s in trial['segments'] if s['kind'] == 'step']
            interrupted = trial.get('interrupted_segment')
            if interrupted and interrupted['kind'] == 'step':
                steps.append(interrupted)
            if not trial['completed']:
                label += ' (aborted)'
            if not steps:
                continue
            for index, segment in enumerate(steps):
                direction = 1 if segment['target'] > segment['start'] else -1
                times = [f['t'] * 1000 for f in segment['frames']]
                errors = [(f['position'] - segment['target']) * direction
                          for f in segment['frames']]
                steady_extent = max(steady_extent, max(
                    (abs(e) for t, e in zip(times, errors, strict=True) if t >= 800),
                    default=0))
                axis[0].plot(times, errors, color=color, alpha=.7,
                             label=label if index == 0 else None)
                axis[1].plot(times, errors, color=color, alpha=.7)
            summary = trial.get('summary') or dict.fromkeys(result['selected']['summary'])
            rows.append({'joint': joint, 'profile': label, **trial['settings'], **summary})
        for a in axis:
            a.axhline(0, color='#666666', linewidth=.8)
            a.set_xlabel('Time after command (ms)')
            a.set_ylabel('Directed target error (tick)')
            a.grid(alpha=.2)
        axis[0].set_title(joint + ' / 5-degree bidirectional steps')
        axis[0].set_xlim(0, 500)
        axis[0].legend()
        axis[1].set_title('Settled error, all repeats')
        axis[1].set_xlim(800, 1200)
        axis[1].set_ylim(-steady_extent - 1, steady_extent + 1)
    for extension in ('png', 'svg', 'pdf'):
        figure.savefig(output / f'response.{extension}', dpi=150)
    plt.close(figure)
    with (output / 'comparison.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    atomic_json(output / 'summary.json', {j: {
        'settings': r['selected']['settings'], 'summary': r['selected']['summary'],
        'hold_jitter_ticks': r['selected'].get('hold_jitter_ticks'),
        'hold_peak_error_ticks': r['selected'].get('hold_peak_error_ticks')}
        for j, r in measured.items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', default='follower')
    parser.add_argument('--joints', nargs='+')
    parser.add_argument('--anchor', type=Path,
                        help='Earlier result fixing the original pose envelope')
    parser.add_argument('--search-amplitude', type=float, choices=(3., 5.), default=3.)
    parser.add_argument('--profiles', type=Path, help='Per-joint register overrides for refinement')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--report', type=Path, nargs='+')
    args = parser.parse_args()
    if args.report:
        report_curves(args.report, args.output)
        return
    if not args.execute:
        parser.error('--execute is required; all joints must already be holding a clear pose')
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)
    run(args)


if __name__ == '__main__':
    main()
