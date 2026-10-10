"""Remote height sampling and application of a locally fitted candidate; no physics engine."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from lerobot_arm_control.calibration.angle_calibration import AngleCalibration
from lerobot_arm_control.calibration.height_records import (
    ANGLES,
    BASE,
    CANDIDATE,
    DATA,
    apply_candidate,
    check_samples,
    digest,
    read_sample,
    save_json,
)
from lerobot_arm_control.robot.so101.model_metadata import MODEL
from lerobot_arm_control.transport.realtime import JOINTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('collect', 'resample', 'apply'))
    parser.add_argument('--count', type=int, default=10)
    parser.add_argument('--data', type=Path, default=DATA)
    parser.add_argument('--poses', type=int, nargs='+', help='One-based sample slots to remeasure')
    args = parser.parse_args()
    if args.count < 10:
        parser.error('--count must be at least 10')
    if args.action == 'apply':
        candidate = json.loads(CANDIDATE.read_text())
        if candidate.get('dataset_sha256') != digest(args.data):
            raise ValueError('Samples changed after fitting; fit again before applying')
    else:
        dataset = json.loads(args.data.read_text()) if args.data.exists() else {
            'version': 1, 'samples': []}
        samples = dataset['samples']
        check_samples(samples)
        if args.action == 'resample' and (not args.poses
                or len(set(args.poses)) != len(args.poses)
                or any(i < 1 or i > len(samples) for i in args.poses)):
            parser.error('resample needs distinct existing --poses numbers, e.g. 3 5 7')
        if args.action in ('collect', 'resample'):
            from lerobot_arm_control.core.config import load_arms
            from lerobot_arm_control.robot.so101.driver import connected
            from lerobot_arm_control.runtime.tabletop_robot import calibration_identity

            print('Read-only sampling. Support the arm; move only after torque is released.\n'
                  'Lightly touch the fixed finger to the desk or a known-height gauge; do not\n'
                  'press/bend it. Vary near/far reach, wrist pitch and roll.\n'
                  'Enter height in mm (Enter = 0 mm), q = save and quit.', flush=True)
            arm = load_arms()['follower']
            with connected(arm) as bus:
                identity = calibration_identity(bus, arm, MODEL)
                calibration = AngleCalibration(ANGLES, identity, use_kinematic=False)
                if calibration.count() != len(JOINTS):
                    raise ValueError('All six original joint calibrations are required')
                remaining = [i-1 for i in args.poses] if args.action == 'resample' else []
                while remaining or (args.action == 'collect' and len(samples) < args.count):
                    index = remaining[0] if remaining else len(samples)
                    value = input(f'Pose {index+1}, stopped height [0 mm]: ')
                    if value.strip().lower() == 'q':
                        save_json(args.data, dataset)
                        return
                    try:
                        sample = read_sample(bus, calibration, float(value or 0)/1000,
                                             json.loads(BASE.read_text()))
                        if any(np.linalg.norm(np.subtract(sample['qpos'][:5], s['qpos'][:5]))
                               < .08 for i, s in enumerate(samples) if i != index):
                            raise ValueError('Pose too similar; change reach or wrist orientation')
                        if remaining:
                            dataset.setdefault('superseded', []).append(
                                {'pose': index+1, 'sample': samples[index]})
                            samples[index] = sample
                            remaining.pop(0)
                        else:
                            samples.append(sample)
                        save_json(args.data, dataset)
                        print(f'Saved {len(samples)} poses; encoder-only reads, no motor writes.')
                    except ValueError as exc:
                        print(f'Sample rejected: {exc}', flush=True)
        if args.action in ('collect', 'resample'):
            print('Samples saved; run fit on the local simulation host.', flush=True)
            return
    print(json.dumps({'offsets_degrees': {j: math.degrees(v) for j, v in
                                         candidate['offsets_rad'].items()},
                      'validation': candidate['validation'], 'candidate': str(CANDIDATE)},
                     indent=2), flush=True)
    if args.action == 'apply':
        path = apply_candidate(candidate)
        print(f'Applied {path}; restart control/viewer to load corrected mappings.')


if __name__ == "__main__":
    main()
