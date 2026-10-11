"""White acquisition leader -> black follower demo using the saved measured desktop."""

import argparse
import json
import math
import queue
import tempfile
import time
from pathlib import Path

from lerobot_arm_control.control.follow import JointFollower, load_follow_config
from lerobot_arm_control.core.files import atomic_json
from lerobot_arm_control.core.paths import resource_path, workspace_root
from lerobot_arm_control.runtime.realtime_client import Link
from lerobot_arm_control.transport.realtime import JOINTS
from lerobot_arm_control.visualization.tabletop_scene import scene_arms, scene_xml


def demo_scene(snapshot, config):
    if (snapshot/'state.json').exists():
        state = json.loads((snapshot/'state.json').read_text())
        scene_config = json.loads((snapshot/'calibration.json').read_text())['config']
    else:
        source = workspace_root()/'configs/scenes/tabletop-current.json'
        parameters = json.loads(source.read_text())
        scene_config = parameters['config']
        state = {key: parameters[key] for key in ('objects', 'models', 'wood_height_measured')}
        state['secondary_layouts'] = parameters['table']['secondary_layouts']
        state['base_registrations'] = {name: arm['registration']
                                       for name, arm in parameters['arms'].items()}
        poses = {name: {'status': 'live', 'qpos': arm['qpos'],
                        'registration': arm['registration']}
                 for name, arm in parameters['arms'].items()}
        state['robot'] = poses[scene_config['robot']['arm']]
        state['robot_feedback'] = poses
    entries = [scene_config['robot'], *scene_config.get('additional_robots', [])]
    for entry in entries:
        entry.setdefault('simulation', {})['end_effector'] = (
            'leader_handle' if entry['arm'] == config['leader'] else 'gripper')
    arms = {arm['arm']: arm for arm in scene_arms(state, scene_config)}
    if config['leader'] not in arms or config['follower'] not in arms:
        raise ValueError('saved scene must contain both localized arms')
    return state, scene_config, arms


def run(args):
    import mujoco

    config = load_follow_config()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    state, scene_config, arms = demo_scene(args.snapshot, config)
    keys = queue.Queue()
    link, viewer = None, None
    started, frames, saved = time.monotonic(), [], False
    initial = [0, -.4, .6, .1, 0, math.radians(20)]
    leader_qpos = list(initial)
    follower_qpos = list(initial)
    mapping = JointFollower(config, follower_qpos)
    selected, notice, hello = 0, 'Synthetic leader; motor writes OFF', None
    try:
        if args.source == 'hardware':
            link = Link(args.host, args.remote_root, 'hardware', hardware_control=args.execute,
                        hardware_worker='follow_server')
            while link.latest is None:
                if link.error or link.process.poll() is not None:
                    raise RuntimeError(link.error or 'follow worker exited')
                if time.monotonic()-started > 20:
                    raise TimeoutError('no dual-arm feedback within 20 seconds')
                time.sleep(.02)
            hello = link.snapshot()[0]
            if hello['config'] != config:
                raise ValueError('Local/remote follow parameters differ; '
                                 'deploy configs/follow.json')
        with tempfile.TemporaryDirectory(prefix='lerobot-follow-') as temp:
            # Textures resolve against the snapshot; mesh paths are explicit local source paths.
            model_path = Path(temp)/'scene.xml'
            xml = scene_xml(state, scene_config, resource_path('assets/so101/assets'))
            marker = args.snapshot.resolve()/'marker.png'
            if not marker.exists():
                import cv2

                marker = Path(temp)/'marker.png'
                dictionary = cv2.aruco.getPredefinedDictionary(
                    getattr(cv2.aruco, scene_config['dictionary']))
                cv2.imwrite(str(marker), cv2.aruco.generateImageMarker(
                    dictionary, scene_config['marker_id'], 256))
            xml = xml.replace('file="marker.png"', f'file="{marker}"')
            xml = xml.replace('file="appearance/', f'file="{args.snapshot.resolve()}/appearance/')
            model_path.write_text(xml)
            model = mujoco.MjModel.from_xml_path(str(model_path))
            data = mujoco.MjData(model)
            addresses = {name: [model.joint(arms[name]['prefix']+joint).qposadr[0]
                                for joint in JOINTS]
                         for name in (config['leader'], config['follower'])}
            if not args.headless:
                from lerobot_arm_control.visualization.control_viewer import ControlViewer

                viewer = ControlViewer(model, data, keys.put,
                                       'SO101 leader handle -> follower demo',
                                       hold_on_focus_loss=args.execute)
                viewer.cam.lookat[:] = [-.32, .03, .14]
                viewer.cam.distance = 1.2
                viewer.cam.azimuth, viewer.cam.elevation = 125, -28
            previous, last_ping = time.monotonic(), 0
            while (viewer is None or viewer.is_running()) and (
                    args.duration is None or time.monotonic()-started < args.duration):
                now = time.monotonic()
                elapsed = now-previous
                previous = now
                if link:
                    if link.error or link.process.poll() is not None:
                        raise RuntimeError(link.error or 'follow worker exited')
                    _, packet, received, *_ = link.snapshot()
                    age = now-received
                    if age > config['max_sample_age_seconds']:
                        raise TimeoutError('stale follow stream; closing worker to hold follower')
                    leader_qpos = packet['leader_qpos']
                    follower_qpos = packet['qpos'] if args.execute else packet['target_qpos']
                    notice = packet['notice']
                    if hello['leader']['mode'] != 'independent':
                        notice += '\nTEMP shared calibration: white angles unverified'
                    if now-last_ping > .2:
                        link.send('ping')
                        last_ping = now
                else:
                    if args.auto:
                        leader_qpos = [initial[i]+(.22 if i < 5 else .15)*math.sin(
                            (now-started)*.7+i*.5) for i in range(6)]
                    follower_qpos = mapping.step(leader_qpos, elapsed)
                while not keys.empty():
                    key = keys.get_nowait()
                    if 49 <= key <= 54:
                        selected = key-49
                    elif key in (265, 264) and not link and not args.auto:
                        leader_qpos[selected] += math.radians(2)*(1 if key == 265 else -1)
                    elif key in (32, 69):
                        if link:
                            link.send('hold' if key == 32 else 'resume')
                        elif key == 32:
                            mapping.hold(follower_qpos)
                        else:
                            mapping.resume(follower_qpos)
                data.qpos[addresses[config['leader']]] = leader_qpos
                data.qpos[addresses[config['follower']]] = follower_qpos
                mujoco.mj_forward(model, data)
                if viewer:
                    viewer.set_texts([(0, 0, '\n'.join([
                        f"Leader: {config['leader']} | acquisition handle + trigger",
                        f"Follower: {config['follower']} | gripper",
                        ('REAL FOLLOWER' if args.execute
                         else 'SIMULATED FOLLOWER | motor writes OFF'),
                        notice, 'E: start/resume | Space: hold',
                        'Synthetic: 1-6 select, Up/Down move (without --auto)',
                        f'Selected: {JOINTS[selected]}',
                        'Leader deg: '+str([round(math.degrees(q), 1) for q in leader_qpos]),
                        'Follower deg: '+str([round(math.degrees(q), 1) for q in follower_qpos]),
                    ]))])
                    viewer.sync(args.screenshot if args.screenshot and not saved else None)
                    saved = True
                frames.append({'time_seconds': now-started, 'leader_qpos': list(leader_qpos),
                               'follower_qpos': list(follower_qpos)})
                if len(frames) > 10000:
                    del frames[:1000]
                time.sleep(max(0, 1/config['hz']-(time.monotonic()-now)))
    finally:
        try:
            if viewer:
                viewer.close()
        finally:
            if link:
                link.close()
            atomic_json(args.log, {'version': 1, 'source': args.source, 'execute': args.execute,
                                   'config': config, 'hello': hello, 'frames': frames})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', choices=('hardware', 'synthetic'), default='hardware')
    parser.add_argument('--execute', action='store_true',
                        help='Enable calibrated real follower with E')
    parser.add_argument('--host', default='lerobot')
    parser.add_argument('--remote-root', default='/home/xy/Research/lerobot')
    parser.add_argument('--snapshot', type=Path,
                        default=workspace_root()/'data/tabletop/current-scene')
    parser.add_argument('--auto', action='store_true', help='Animate synthetic leader')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--screenshot', type=Path)
    parser.add_argument('--log', type=Path, default=workspace_root()/'logs/follow-latest.json')
    args = parser.parse_args()
    if args.execute and (args.source != 'hardware' or args.headless):
        parser.error('--execute requires hardware input and the interactive window')
    if args.auto and args.source != 'synthetic':
        parser.error('--auto requires --source synthetic')
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be a finite positive number')
    run(args)


if __name__ == '__main__':
    main()
