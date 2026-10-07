"""Shared read-only height records and validation of locally fitted corrections."""

import hashlib
import json
import math
import statistics
import time
from pathlib import Path

import numpy as np

from .angle_calibration import AngleCalibration, validate_correction
from .realtime import JOINTS

ROOT = Path(__file__).resolve().parents[2]
ANGLES = ROOT/'data/calibration/follower_mujoco.json'
BASE = ROOT/'data/calibration/follower_table_base.json'
DATA = ROOT/'data/calibration/height_samples.json'
CANDIDATE = ROOT/'data/calibration/follower_height_candidate.json'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)
def read_sample(bus, calibration, height_m, base_record, *, sleep=time.sleep):
    """Every bus operation is a read; opening/closing is handled by driver.connected."""
    if not math.isfinite(height_m) or not 0 <= height_m <= .15:
        raise ValueError('Height must be between 0 and 150 mm')
    samples = []
    for _ in range(12):
        raw = bus.sync_read('Present_Position', normalize=False)
        status = bus.sync_read('Status', normalize=False)
        if any(status.values()):
            raise ValueError('Motor fault: sample rejected')
        samples.append({'raw': raw, 'status': status, 'sampled_unix_ns': time.time_ns()})
        sleep(.1)
    spans = {joint: max(s['raw'][joint] for s in samples)
             - min(s['raw'][joint] for s in samples) for joint in JOINTS}
    if max(spans.values()) > 4:
        raise ValueError('Arm moved during sampling; hold still and repeat')
    raw = {joint: statistics.median(s['raw'][joint] for s in samples) for joint in JOINTS}
    return {'measured_height_m': height_m, 'qpos': calibration.angles(raw, [0.]*6),
            'stable': True, 'raw_span_ticks': spans, 'samples': samples,
            'writes_performed': False, 'joint_calibration_sha256': digest(calibration.path),
            'base_calibration': base_record}


def check_samples(samples, angles_path=ANGLES, base_path=BASE):
    angles_hash = digest(angles_path)
    saved_angles = json.loads(Path(angles_path).read_text())
    calibration = AngleCalibration(Path(angles_path), saved_angles['fingerprint'],
                                   use_kinematic=False)
    if calibration.count() != len(JOINTS):
        raise ValueError('All six original joint calibrations are required')
    base = json.loads(Path(base_path).read_text())
    if not base['pose'].get('fixed_to_table') or not base['pose'].get('calibrated'):
        raise ValueError('A measured fixed base calibration is required')
    for sample in samples:
        if (sample.get('stable') is not True or sample.get('writes_performed') is not False
                or sample.get('joint_calibration_sha256') != angles_hash
                or sample.get('base_calibration') != base):
            raise ValueError('Sample calibration changed or sampling was not stable/read-only')
        if (len(sample.get('samples', [])) < 9 or max(
                sample.get('raw_span_ticks', {}).values(), default=math.inf) > 4
                or any(any(s['status'].values()) for s in sample['samples'])):
            raise ValueError('Sample has insufficient stable or fault-free encoder evidence')
        qpos = np.asarray(sample.get('qpos', []), float)
        height = sample.get('measured_height_m')
        if (qpos.shape != (6,) or not np.isfinite(qpos).all()
                or type(height) not in (int, float) or not math.isfinite(height)
                or not 0 <= height <= .15):
            raise ValueError('Invalid measured height or encoder angles')
        raw = {joint: statistics.median(s['raw'][joint] for s in sample['samples'])
               for joint in JOINTS}
        if any(max(s['raw'][joint] for s in sample['samples'])
               - min(s['raw'][joint] for s in sample['samples']) > 4 for joint in JOINTS):
            raise ValueError('Encoder evidence moved during sampling')
        if not np.allclose(qpos, calibration.angles(raw, [0.]*6), atol=1e-10, rtol=0):
            raise ValueError('Recorded angles do not match original encoder calibration')
    return base

def apply_candidate(candidate, *, angles_path=ANGLES, base_path=BASE):
    saved = json.loads(Path(angles_path).read_text())
    validate_correction(candidate, digest(angles_path), saved['fingerprint'])
    if digest(base_path) != candidate['base_calibration_sha256']:
        raise ValueError('Base changed after fitting')
    destination = Path(angles_path).with_name(Path(angles_path).stem+'_kinematic.json')
    if destination.exists():
        backup = destination.with_name(destination.stem+f'-backup-{time.time_ns()}.json')
        backup.write_bytes(destination.read_bytes())
    save_json(destination, candidate)
    return destination
