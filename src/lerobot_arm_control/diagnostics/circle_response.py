"""Full-rate encoder/model circle metrics and response figures, after hardware holds."""

import json
from pathlib import Path

import mujoco
import numpy as np

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.core.files import atomic_json, file_digest
from lerobot_arm_control.core.paths import resource_path
from lerobot_arm_control.robot.so101.joints import JOINTS


def response_report(trace_path, plan_path, output, *, plots=True):
    trace = json.loads(Path(trace_path).read_text())
    plan = json.loads(Path(plan_path).read_text())
    calibration_path = resource_path('data/calibration/follower_mujoco.json')
    if (plan.get('joint_calibration_sha256') is not None
            and plan['joint_calibration_sha256'] != file_digest(calibration_path)):
        raise ValueError('response calibration changed; use the original reference')
    metadata = trace.get('metadata', {})
    if (metadata.get('joint_calibration_sha256', file_digest(calibration_path))
            != file_digest(calibration_path)):
        raise ValueError('response trace requires its original calibration')
    if metadata.get('plan_sha256', file_digest(plan_path)) != file_digest(plan_path):
        raise ValueError('response plan differs from the executed plan')
    frames = [f for f in trace['frames'] if (f.get('circle') or {}).get('phase') == 'running']
    if len(frames) < 20:
        raise ValueError('insufficient running control frames')
    calibration = AngleCalibration(calibration_path,
                                   json.loads(calibration_path.read_text())['fingerprint'])
    actual = np.array([calibration.angles(f['raw'], [0.]*6) for f in frames])
    reference = np.array([[f['nominal_qpos'][j] for j in JOINTS] for f in frames])
    info = frames[0]['circle']
    time = np.array([f['circle']['elapsed_seconds']-f['circle']['entry_seconds'] for f in frames])
    steady = ((time > info['ramp_seconds']+.25)
              & (time < info['laps']*info['period_seconds']-.25))
    if np.any(np.diff(time) <= 0):
        raise ValueError('invalid control clock')
    has_steady = steady.sum() >= 20
    model = mujoco.MjModel.from_xml_path(str(resource_path('assets/so101/so101.xml')))
    data = mujoco.MjData(model)

    def forward(values):
        result = []
        for q in values:
            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            result.append(data.site('gripperframe').xpos.copy())
        return np.array(result)

    xyz, desired = forward(actual), forward(reference)
    axes, normal = ((0, 1), 2) if plan['plane'] == 'XY' else ((0, 2), 1)
    offset = xyz-np.array(plan['center_m'])
    radial = np.linalg.norm(offset[:, axes], axis=1)
    geometric = np.hypot(radial-plan['radius_m'], offset[:, normal])
    tracking = np.linalg.norm(xyz-desired, axis=1)
    phase = np.rad2deg(np.unwrap(np.arctan2(offset[:, axes[1]], offset[:, axes[0]])))
    # Resample the small clock jitter before spectral scoring. Do not score the
    # planned startup/braking as physical vibration or hide it from the plots.
    ts = time[steady]
    uniform = np.linspace(ts[0], ts[-1], len(ts)) if has_steady else np.array([])
    frequency = (np.fft.rfftfreq(len(ts), uniform[1]-uniform[0])
                 if has_steady else np.array([0.]))
    window = np.hanning(len(ts))
    weight = np.full(len(frequency), 2.)
    weight[0] = 1
    if len(ts) % 2 == 0:
        weight[-1] = 1

    def high_frequency_rms(values):
        if not has_steady:
            return None
        values = np.interp(uniform, ts, values[steady])
        values -= np.polyval(np.polyfit(uniform, values, 1), uniform)
        spectrum = np.fft.rfft(values*window)
        selected = frequency >= 3.
        return float(np.sqrt(np.sum(abs(spectrum[selected])**2*weight[selected])
                             /(len(ts)*np.sum(window**2))))

    filtered_phase = np.convolve(phase, np.ones(7)/7, mode='valid')
    speed_time = np.convolve(time, np.ones(7)/7, mode='valid')
    speed = np.gradient(filtered_phase, speed_time)
    speed_steady = ((speed_time > info['ramp_seconds']+.4)
                    & (speed_time < info['laps']*info['period_seconds']-.4))
    errors = actual-reference
    per_joint = {j: {
        'tracking_rms_degrees': float(np.rad2deg(np.sqrt(np.mean(errors[:, i]**2)))),
        'tracking_max_degrees': float(np.rad2deg(abs(errors[:, i]).max())),
        'hf_error_rms_degrees': (float(np.rad2deg(high_frequency_rms(errors[:, i])))
                                 if has_steady else None),
        'overshoot_degrees': float(np.rad2deg(max(0, actual[:, i].max()-reference[:, i].max(),
                                                  reference[:, i].min()-actual[:, i].min()))),
    } for i, j in enumerate(JOINTS)}
    result = {
        'running_control_frames': len(frames), 'steady_control_frames': int(steady.sum()),
        'requested_angular_speed_deg_s': 360/info['period_seconds'],
        'steady_phase_fit_speed_deg_s': (float(np.polyfit(ts, phase[steady], 1)[0])
                                         if has_steady else None),
        'speed_7sample_p05_p50_p95_deg_s': (np.percentile(
            speed[speed_steady], [5, 50, 95]).tolist() if speed_steady.sum() >= 20 else None),
        'control_hz': float(1/np.mean(np.diff(time))),
        'max_running_control_period_ms': float(np.diff(time).max()*1000),
        'circle_rms_mm': float(np.sqrt(np.mean(geometric**2))*1000),
        'circle_max_mm': float(geometric.max()*1000),
        'tracking_rms_mm': float(np.sqrt(np.mean(tracking**2))*1000),
        'tracking_max_mm': float(tracking.max()*1000),
        'tcp_hf_error_rms_mm': float(np.linalg.norm([
            high_frequency_rms((xyz-desired)[:, i]) for i in range(3)])*1000)
                                if has_steady else None,
        'per_joint': per_joint,
        'status_flags': sum(bool(v) for f in frames for v in f['telemetry']['Status'].values()),
        'plan_sha256': file_digest(plan_path), 'trace_sha256': file_digest(trace_path),
        'joint_calibration_sha256': file_digest(calibration_path),
        'control_config_sha256': trace['control_config_sha256'],
        'profile': metadata.get('position_feedback_profile'),
        'final_native_parameters': metadata.get('final_native_parameters'),
        'method': 'Encoder + calibrated CAD; not independent optical measurement. '
                  'Jitter is steady tracking residual >=3Hz, detrended, Hann-windowed; '
                  'speed percentiles use a seven-control-sample average. '
                  'Null steady metrics mean the finite motion had no adequate steady segment.',
    }
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    atomic_json(output.with_suffix('.json'), result)
    if plots:
        _plots(output, time, xyz, desired, offset, normal, axes, speed_time, speed,
               actual, reference, result)
    return result


def _plots(output, time, xyz, desired, offset, normal, axes, speed_time, speed,
           actual, reference, result):
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, panels = plt.subplots(2, 2, figsize=(12, 8))
    ax = panels[0, 0]
    ax.plot(*desired[:, axes].T*100, '--', color='#ed7137', label='Target')
    ax.plot(*xyz[:, axes].T*100, color='#129d8b', lw=.8, label='Encoder + model')
    ax.set_aspect('equal')
    ax.set(xlabel='Base X (cm)', ylabel=f'Base {"Y" if normal == 2 else "Z"} (cm)',
           title='TCP circle')
    ax.legend()
    panels[0, 1].plot(speed_time, speed, color='#129d8b', lw=.9)
    panels[0, 1].axhline(result['requested_angular_speed_deg_s'], color='#ed7137', ls='--')
    fitted_speed = result['steady_phase_fit_speed_deg_s']
    speed_label = f'{fitted_speed:.2f} deg/s' if fitted_speed is not None else 'no steady segment'
    panels[0, 1].set(xlabel='Time on circle (s)', ylabel='Circle angular speed (deg/s)',
                     title=f'Steady fit: {speed_label}')
    panels[1, 0].plot(time, np.linalg.norm(xyz-desired, axis=1)*1000, color='#ed7137', lw=.8)
    panels[1, 0].set(xlabel='Time on circle (s)', ylabel='TCP tracking error (mm)',
                     title=f'RMS {result["tracking_rms_mm"]:.2f} mm')
    panels[1, 1].plot(time, offset[:, normal]*1000, color='#5263bd', lw=.8)
    jitter = result['tcp_hf_error_rms_mm']
    jitter_label = f'{jitter:.2f} mm RMS' if jitter is not None else 'no steady segment'
    panels[1, 1].set(xlabel='Time on circle (s)', ylabel='Plane deviation (mm)',
                     title=f'TCP residual >=3 Hz: {jitter_label}')
    for ax in panels.flat:
        ax.grid(alpha=.2)
    fig.suptitle('Remote finite circle | calibrated encoder estimates')
    fig.tight_layout()
    fig.savefig(output.with_suffix('.png'), dpi=160)
    plt.close(fig)
    fig, panels = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    for i, (ax, joint) in enumerate(zip(panels.flat, JOINTS)):
        ax.plot(time, np.rad2deg(reference[:, i]), color='#ed7137', lw=1.5, label='Target')
        ax.plot(time, np.rad2deg(actual[:, i]), color='#129d8b', lw=.8, label='Feedback')
        jitter = result['per_joint'][joint]['hf_error_rms_degrees']
        label = f'{jitter:.3f} deg' if jitter is not None else 'no steady segment'
        ax.set(title=f'{i+1}: {joint} | HF RMS {label}', ylabel='Angle (deg)')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    for ax in panels[-1]:
        ax.set_xlabel('Time on circle (s)')
    fig.tight_layout()
    fig.savefig(output.with_name(output.name+'-joints').with_suffix('.png'), dpi=160)
    plt.close(fig)
