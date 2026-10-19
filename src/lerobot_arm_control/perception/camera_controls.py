"""Apply saved Linux exposure controls and verify their actual device readback."""

import subprocess


def apply_v4l2_controls(device, controls):
    if not controls:
        return None
    if (not set(controls) <= {'auto_exposure', 'exposure_time_absolute', 'gain'}
            or any(type(value) is not int for value in controls.values())):
        raise ValueError('Unsupported camera exposure controls')
    subprocess.run(['v4l2-ctl', '-d', device, '--set-ctrl',
                    ','.join(f'{key}={value}' for key, value in controls.items())],
                   check=True, capture_output=True, timeout=5)
    readback = subprocess.run(['v4l2-ctl', '-d', device, '--get-ctrl', ','.join(controls)],
                             check=True, capture_output=True, text=True, timeout=5).stdout
    actual = dict(line.strip().split(':', 1) for line in readback.splitlines())
    if any(int(actual[key].strip().split()[0]) != value for key, value in controls.items()):
        raise ValueError('Camera exposure readback differs from saved calibration controls')
    return readback
