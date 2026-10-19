"""Calibrate the camera/table/arm-base relationship without opening a motor bus."""

import argparse
import json
import math
from datetime import datetime
from pathlib import Path

from lerobot_arm_control.core.files import atomic_json
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.core.runtime_config import add_remote_arguments


def calibration_text(config, state, frame_age):
    """Show spatial measurements and acceptance progress, without joint/task controls."""
    valid = frame_age < .5 and state.get('marker_visible')
    lines = ['Table-relative calibration | camera only',
             f"Table anchor ID {config['marker_id']}: {'TRACKING' if valid else 'LOST / STALE'}",
             '+X marker right | +Y marker up | +Z above table']
    if valid and not state.get('anchor_visible'):
        lines.append('Primary anchor hidden: base calibration samples paused')
    camera = state.get('camera_position')
    if camera is not None:
        lines.append('Camera in table frame (mm): '+', '.join(f'{v*1000:.1f}' for v in camera))
    entries = [config.get('robot', {}), *config.get('additional_robots', [])]
    poses = state.get('base_registrations', {})
    for entry in entries:
        if not entry.get('enabled'):
            continue
        name = entry['arm']
        pose = poses.get(name, {})
        status = pose.get('status', 'waiting for marker')
        progress = (" (accepted)" if status == "fixed" else
                    f" | samples {pose.get('fixed_calibration_samples', 0)}/"
                    f"{pose.get('fixed_calibration_required', '?')}")
        lines.append(f"{name}: {status}{progress}")
        if pose.get('calibrated') and pose.get('position') is not None:
            xyz = ', '.join(f'{v*1000:.1f}' for v in pose['position'])
            q = pose['quaternion']
            yaw = math.degrees(2*math.atan2(q[3], q[0]))
            lines.append(f'  XYZ (mm): {xyz} | yaw {yaw:.1f} deg')
        if pose.get('rejection_reason'):
            lines.append('  '+pose['rejection_reason'])
        if status == 'markers_lost':
            expected = ', '.join(f"{m['dictionary']} ID {m['id']}"
                                 for m in entry.get('markers', []))
            lines.append('  No verified base marker: '+expected)
    for pair in state.get('base_distances', []):
        lines.append(' / '.join(pair['arms'])+f": {pair['planar_distance_m']*1000:.1f} mm apart")
    lines.append('Joint angles are not sampled | Esc: close and save session')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_remote_arguments(parser)
    parser.add_argument('--output', type=Path, help='New calibration session directory')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--screenshot', type=Path)
    parser.add_argument('--fps', type=float, default=5)
    parser.add_argument('--reset-table', action='store_true',
                        help='Also rebuild the static table layout after moving table markers')
    parser.add_argument('--capture', type=Path,
                        help='Offline: fit the two-table-marker layout from saved observations')
    args = parser.parse_args()
    if not 0 < args.fps <= 30:
        parser.error('fps must be in (0, 30]')
    if args.duration is not None and args.duration <= 0:
        parser.error('duration must be positive')
    if args.capture and (args.reset_table or args.headless or args.screenshot
                         or args.duration is not None):
        parser.error('capture fitting cannot be combined with live calibration options')
    if args.output is None:
        args.output = workspace_root()/'logs'/('table-calibration-'+
                                             datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    if args.output.exists():
        parser.error('output must be a new session directory')
    if args.capture:
        from lerobot_arm_control.calibration.table_reference_calibration import fit_capture
        from lerobot_arm_control.perception.tabletop_perception import read_config

        result = fit_capture(args.capture, read_config(workspace_root()/'configs/tabletop.json'))
        args.output.mkdir(parents=True)
        target = args.output/'table-reference.json'
        atomic_json(target, result)
        print(json.dumps({'output': str(target), 'quality': result['quality']}))
        return
    from lerobot_arm_control.runtime import tabletop_client

    # Spatial registration uses visible rigid markers. Joint readings and motor
    # control are unnecessary, including when either arm is unpowered.
    args.calibration_mode = True
    args.camera_only = True
    args.hardware_control = args.push_t = False
    args.recalibrate_base = True
    args.recalibrate_table = args.reset_table
    tabletop_client.run(args)


if __name__ == '__main__':
    main()
