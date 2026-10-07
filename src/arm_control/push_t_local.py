"""Local camera-driven Push-T planning; the remote worker only executes checked segments."""

import base64
import copy
import json
import math
import shutil
import threading
import time
from pathlib import Path

import mujoco
import numpy as np

from .parking import smooth_fraction
from .push_t import Push, PushSimulation, errors, rotation
from .push_t_motor import (
    calibrated_base,
    joint_limit_issue,
    joint_segments,
    observation_issue,
    recovery_issue,
    same_scene,
)
from .tabletop_scene import ROOT


class ShortPushSimulation(PushSimulation):
    fast_free_approach = True
    interpolate = staticmethod(smooth_fraction)

    def candidates(self, current):
        candidates = super().candidates(current)
        for action in candidates:
            action.distance = min(action.distance, .004)
        return candidates

    def play_action(self, action, model, data):
        segments = joint_segments(data.ctrl, action, self.arm.limits)
        skipped = 0
        if self.fast_free_approach and data is self.roll_data:
            skipped = self.free_approach(action.approach_count, segments, model, data)
        if skipped:
            self.step(data.ctrl.copy(), .12, model, data)
        # Contact prediction and actual simulation retain the real bus interpolation and speed.
        for segment in segments[skipped:]:
            self.step(np.asarray(segment['end']), segment['seconds'], model, data)
        self.step(data.ctrl.copy(), .12, model, data)

    def free_approach(self, count, segments, model, data):
        """Skip dynamics only for prediction of a fully collision-checked free transfer."""
        if not count:
            return 0
        wood = model.body('wood_t').id

        def touching():
            for contact in data.contact:
                a, b = (model.geom_bodyid[g] for g in (contact.geom1, contact.geom2))
                if wood in (a, b) and 0 in (a, b):
                    continue
                if wood in (a, b) or contact.dist < -.0005:
                    return True
            return False

        if touching():
            return 0
        checkpoint = getattr(self, '_approach_checkpoint', None)
        if checkpoint is None:
            checkpoint = self._approach_checkpoint = mujoco.MjData(model)
        mujoco.mj_copyData(checkpoint, model, data)
        for segment in segments[:count]:
            start, end = np.asarray(segment['start']), np.asarray(segment['end'])
            samples = max(3, int(np.max(np.abs(end-start))/.015)+1)
            for fraction in np.linspace(0, 1, samples):
                data.qpos[self.qadr] = start+(end-start)*fraction
                mujoco.mj_forward(model, data)
                if touching():
                    mujoco.mj_copyData(data, model, checkpoint)
                    return 0
        data.ctrl[:] = end
        data.qvel[self.arm.dadr] = 0
        self.fast_approach_predictions = getattr(self, 'fast_approach_predictions', 0)+1
        return count


class LocalPush:
    def __init__(self, config, robot, output, marker_png, *, execute=False, publish=None):
        self.config, self.robot, self.output = config, robot, Path(output)
        self.execute = execute
        self.publish = publish
        self.height_compensation = float(config.get('robot', {}).get(
            'push_height_compensation_m', 0))
        if not math.isfinite(self.height_compensation):
            raise ValueError('push height compensation must be finite')
        self.marker_png = marker_png
        self.output.mkdir(parents=True, exist_ok=False)
        self.observation_log = (self.output/'observations.jsonl').open('a')
        self.lock = threading.Lock()
        self.changed = threading.Event()
        self.stopped = threading.Event()
        self.state, self.received = None, 0
        self.appearance_state = None
        self.last_visible = None
        self.requested = False
        self.generation = 0
        self.progress = {'phase': 'waiting for observation', 'execution_enabled': execute,
                         'execution_location': 'remote', 'planning_location': 'local',
                         'height_compensation_m': self.height_compensation,
                         'output': str(self.output), 'pushes': 0}
        self.sim = None
        self.planning_started = None
        self.pushes, self.success_frames = 0, 0
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def observe(self, state):
        frozen = copy.deepcopy(state)
        received = time.monotonic()
        recorded = {key: value for key, value in state.items()
                    if key not in ('jpeg', 'annotated_jpeg', 'native_jpeg', 'appearance_pngs')}
        self.observation_log.write(json.dumps(recorded, allow_nan=False)+'\n')
        self.observation_log.flush()
        with self.lock:
            self.state, self.received = frozen, received
            if 'appearance_pngs' in frozen:
                self.appearance_state = frozen
            if observation_issue(frozen, received, received, initialized=True) is None:
                self.last_visible = (frozen, received)
        if self.robot.motor:
            self.robot.motor.observe(frozen, received)
        self.changed.set()

    def command(self, op):
        if op not in ('push_start', 'push_stop'):
            raise ValueError('unknown push command')
        with self.lock:
            if op == 'push_start' and self.requested:
                return
            self.generation += 1
            self.requested = op == 'push_start'
            if self.requested:
                self.pushes, self.success_frames = 0, 0
        if op == 'push_stop' and self.robot.motor:
            self.robot.motor.cancel('operator stop: hold')
        self.changed.set()

    def status(self):
        with self.lock:
            result = dict(self.progress)
            result['start_requested'] = self.requested
            if result['phase'] == 'planning locally' and self.planning_started is not None:
                result['planning_elapsed_s'] = time.monotonic()-self.planning_started
        if self.robot.motor:
            result['motor'] = self.robot.motor.status()
        return result

    def _status(self, phase, **fields):
        with self.lock:
            if phase == 'planning locally':
                self.planning_started = time.monotonic()
            else:
                self.planning_started = None
            self.progress.update(phase=phase, pushes=self.pushes, **fields)

    def _latest(self):
        with self.lock:
            return self.state, self.received, self.requested, self.generation

    def _prepare(self, state):
        source = self.output/'initial_observation'
        source.mkdir()
        (source/'state.json').write_text(json.dumps(state, indent=2))
        (source/'calibration.json').write_text(json.dumps({'config': self.config,
                                                         'robot': self.robot.metadata}, indent=2))
        shutil.copyfile(ROOT/self.config['robot']['joint_calibration'],
                        source/'joint_calibration.json')
        shutil.copytree(ROOT/'assets/so101/assets', source/'robot_assets')
        (source/'marker.png').write_bytes(self.marker_png)
        appearance = source/'appearance'
        appearance.mkdir()
        for filename, data in state['appearance_pngs'].items():
            if Path(filename).name != filename or not filename.endswith('.png'):
                raise ValueError('invalid texture file')
            (appearance/filename).write_bytes(base64.b64decode(data, validate=True))
        (appearance/'appearance.json').write_text(json.dumps(state['appearance'], indent=2))
        self.sim = ShortPushSimulation(source, self.output/'model')

    def _update_model(self, state):
        sim = self.sim
        sim.state = copy.deepcopy(state)
        sim.target = np.r_[state['objects']['black_t']['xy'], state['objects']['black_t']['yaw']]
        sim.initial = np.r_[state['objects']['wood_t']['xy'], state['objects']['wood_t']['yaw']]
        base = state['robot']['registration']
        for model in (sim.model, sim.roll_model):
            model.body('robot_root').pos[:] = base['position']
            model.body('robot_root').quat[:] = base['quaternion']
            model.body('black_t').pos[:2] = sim.target[:2]
            model.body('black_t').quat[:] = [math.cos(sim.target[2]/2), 0, 0,
                                            math.sin(sim.target[2]/2)]
        sim.reset(sim.initial)
        # Real measured positions are the IK seed; no simulated settling drift in joint angles.
        sim.data.qpos[sim.qadr] = state['robot']['qpos']
        sim.data.qvel[:] = 0
        sim.data.ctrl[:] = state['robot']['qpos']
        mujoco.mj_forward(sim.model, sim.data)

    def _execution_action(self, action, initial):
        """Raise command targets once, retaining nominal contact rollouts for planning."""
        if not self.height_compensation:
            return action
        sim, arm = self.sim, self.sim.arm
        nominal = mujoco.MjData(sim.model)
        mujoco.mj_copyData(nominal, sim.model, sim.data)
        mujoco.mj_copyData(arm.data, sim.model, sim.data)
        raised = copy.deepcopy(action)
        raised.joints = []
        previous = np.asarray(initial, float)
        for index, joints in enumerate(action.joints):
            nominal.qpos[sim.qadr] = joints
            mujoco.mj_forward(sim.model, nominal)
            target = nominal.site('push_contact').xpos.copy()
            target[2] += self.height_compensation
            q = arm.solve(target, previous, attempts=8 if index == 0 else 1)
            if q is None or not arm.path_clear(previous, q,
                                               avoid_wood=index < action.approach_count):
                raise ValueError('height-compensated trajectory unreachable or in collision')
            raised.joints.append(q)
            previous = q
        return raised

    def _run(self):
        last_preview = None
        last_action = None
        completed_reference = None
        success_sequence = None
        active = False
        recovering = False
        recovery_attempts = 0
        try:
            with (self.output/'plans.jsonl').open('a') as log:
                while not self.stopped.is_set():
                    self.changed.wait(.1)
                    self.changed.clear()
                    state, received, requested, generation = self._latest()
                    issue = (recovery_issue(state, received, time.monotonic()) if recovering else
                             observation_issue(state, received, time.monotonic(),
                                               initialized=self.sim is not None))
                    if issue:
                        if active and self.robot.motor:
                            if not recovering and recovery_attempts < 2:
                                recovering = self._recover(state, received)
                                if recovering:
                                    recovery_attempts += 1
                                    self._status('clearing camera occlusion', reason=issue)
                                    continue
                            self.robot.motor.cancel(issue)
                            active = recovering = False
                            with self.lock:
                                self.requested = False
                        self._status('waiting for observation', reason=issue)
                        continue
                    if self.sim is None:
                        with self.lock:
                            appearance_state = self.appearance_state
                        if appearance_state is None:
                            self._status('waiting for textures', reason='new session appearance')
                            continue
                        initial = copy.deepcopy(state)
                        initial['appearance_pngs'] = appearance_state['appearance_pngs']
                        self._prepare(initial)
                    if self.robot.motor:
                        motor = self.robot.motor.status()
                        if motor['phase'] in ('pending', 'running'):
                            self._status('clearing camera occlusion' if recovering else
                                         'executing real push', reason=None)
                            continue
                        if active:
                            active = False
                            if motor['phase'] != 'completed':
                                self._status('stopped', reason=motor.get('reason'))
                                with self.lock:
                                    self.requested = False
                                continue
                            if not recovering:
                                self.pushes += 1
                            recovering = False
                            # Wait for a camera frame captured after the last motor waypoint.
                            end_ns = time.time_ns()
                            while not self.stopped.is_set():
                                newer = self._latest()[0]
                                if newer and newer['captured_unix_ns'] > end_ns:
                                    break
                                self.changed.wait(.05)
                                self.changed.clear()
                            continue
                    recovery_attempts = 0
                    seed_issue = joint_limit_issue(state['robot']['qpos'], self.sim.arm.limits)
                    if seed_issue:
                        last_preview = None
                        self._status('waiting for valid joint pose', reason=seed_issue,
                                     executable=False, next_push=None)
                        continue
                    if not requested and completed_reference:
                        if same_scene(completed_reference, state, wood_tolerance=.005):
                            self._status('success from real camera', reason=None)
                            continue
                        completed_reference = None
                    distance, yaw = errors(np.r_[state['objects']['wood_t']['xy'],
                                                 state['objects']['wood_t']['yaw']],
                                           np.r_[state['objects']['black_t']['xy'],
                                                 state['objects']['black_t']['yaw']])
                    self._status('observing', position_error_m=distance, yaw_error_rad=yaw)
                    if requested and distance < .002 and yaw < math.radians(3):
                        if success_sequence != state['sequence']:
                            self.success_frames += 1
                            success_sequence = state['sequence']
                        if self.success_frames >= 3:
                            with self.lock:
                                self.requested = False
                            self._status('success from real camera', reason=None)
                            completed_reference = state
                            result = self.status()
                            (self.output/'result.json').write_text(json.dumps(result, indent=2))
                            if self.publish:
                                self.publish(result)
                        continue
                    self.success_frames = 0
                    success_sequence = None
                    if self.pushes >= 100:
                        self._status('stopped', reason='100 push budget exhausted')
                        with self.lock:
                            self.requested = False
                        continue
                    executable_issue = observation_issue(state, received, time.monotonic(),
                                                          require_base=True, initialized=True)
                    blocker = executable_issue or (None if self.execute else
                                                   'session launched in preview mode')
                    if requested and (not self.execute or executable_issue):
                        self._status('execution blocked', reason=executable_issue or
                                     'session launched in preview mode')
                        with self.lock:
                            self.requested = False
                        requested = False
                    reuse_preview = (last_preview is not None and last_action is not None
                                     and same_scene(last_preview, state))
                    if not requested and reuse_preview:
                        # The most recent preview remains visible; do not recompute a static scene.
                        self._status('preview ready', executable=not bool(blocker), reason=blocker)
                        continue
                    started = time.monotonic()
                    if reuse_preview:
                        action = last_action
                        reference = last_preview
                    else:
                        self._status('planning locally', reason=executable_issue,
                                     executable=False, next_push=None)
                        self._update_model(state)
                        if not self.sim.valid_pose(self.sim.initial):
                            self._status('unreachable', reason='wood outside the valid task region')
                            continue
                        reference = state
                        action = self.sim.plan()
                    current, arrived, still_requested, latest_generation = self._latest()
                    if self.stopped.is_set():
                        break
                    if (observation_issue(current, arrived, time.monotonic(), initialized=True)
                            or not same_scene(reference, current)):
                        self._status('replanning', reason='scene changed during planning')
                        continue
                    if action is None:
                        self._status('unreachable', reason=self.sim.phase)
                        with self.lock:
                            self.requested = False
                        continue
                    try:
                        command_action = self._execution_action(action, current['robot']['qpos'])
                        segments = joint_segments(current['robot']['qpos'], command_action,
                                                  self.sim.arm.limits)
                    except ValueError as exc:
                        self._status('execution blocked', reason=str(exc), executable=False)
                        with self.lock:
                            self.requested = False
                        continue
                    plan = {'sequence': state['sequence'],
                            'captured_unix_ns': state['captured_unix_ns'],
                            'joint_seed': state['robot']['qpos'],
                            'objects': state['objects'], 'base': state['robot']['registration'],
                            'start': action.start.tolist(), 'direction': action.direction.tolist(),
                            'distance_m': action.distance, 'transition': action.transition,
                            'segments': segments, 'planning_seconds': time.monotonic()-started,
                            'height_compensation_m': self.height_compensation,
                            'reused_preview': bool(reuse_preview),
                            'execution_blocker': blocker}
                    log.write(json.dumps(plan, allow_nan=False)+'\n')
                    log.flush()
                    (self.output/'preview.json').write_text(json.dumps(plan, indent=2))
                    if self.publish:
                        self.publish(plan)
                    self._status('preview ready', next_push={key: plan[key] for key in (
                        'start', 'direction', 'distance_m', 'transition', 'planning_seconds')},
                        executable=not bool(blocker), reason=blocker)
                    last_preview = state
                    last_action = action
                    if (requested and still_requested and latest_generation == generation
                            and self.execute and not executable_issue):
                        if self.robot.motor is None:
                            raise RuntimeError('hardware executor unavailable')
                        self.robot.motor.submit(current, segments)
                        last_preview = last_action = None
                        active = True
        except Exception as exc:
            if self.robot.motor:
                self.robot.motor.cancel('planner fault: '+str(exc))
            self._status('error', reason=str(exc))

    def close(self):
        self.stopped.set()
        self.changed.set()
        if self.robot.motor:
            self.robot.motor.cancel('local planner closed: hold')
        self.worker.join(timeout=3)
        self.observation_log.close()

    def _recover(self, state, received):
        """Clear the camera using cached geometry; encoder feedback still closes every move."""
        if recovery_issue(state, received, time.monotonic()):
            return False
        with self.lock:
            saved = self.last_visible
        if not saved or time.monotonic()-saved[1] > 1:
            return False
        reference = copy.deepcopy(saved[0])
        base = reference['robot']['registration']
        if not calibrated_base(base):
            return False
        self.robot.motor.cancel('pause contact stroke for visibility recovery')
        deadline = time.monotonic()+.5
        while self.robot.motor.status()['phase'] in ('running', 'pending', 'stopping'):
            if time.monotonic() > deadline or self.stopped.is_set():
                return False
            self.stopped.wait(.02)
        current, arrived, _, _ = self._latest()
        if recovery_issue(current, arrived, time.monotonic()):
            return False
        if joint_limit_issue(current['robot']['qpos'], self.sim.arm.limits):
            return False
        reference['robot']['qpos'] = current['robot']['qpos']
        self._update_model(reference)
        sim, arm = self.sim, self.sim.arm
        mujoco.mj_copyData(arm.data, sim.model, sim.data)
        seed = sim.data.qpos[sim.qadr].copy()
        site = sim.data.site('push_contact').xpos.copy()
        outward = site[:2]-sim.initial[:2]
        if np.linalg.norm(outward) < .001:
            return False
        outward /= np.linalg.norm(outward)
        options = [[site+[0, 0, .025]]]
        for turn in (0, -.6, .6):
            release = site+np.r_[rotation(turn)@outward*.015, 0]
            options.append([release, release+[0, 0, .025]])
        for points in options:
            joints, previous = [], seed
            for target in points:
                q = arm.solve(target, previous, attempts=8 if not joints else 1)
                if q is None or not arm.path_clear(previous, q, avoid_wood=True):
                    break
                joints.append(q)
                previous = q
            if len(joints) != len(points):
                continue
            action = Push(site[:2], outward, 0, joints=joints,
                          approach_count=len(joints), transition='visibility recovery')
            segments = joint_segments(seed, action, arm.limits)
            if sum(s['seconds'] for s in segments) > 7:
                continue
            self.robot.motor.submit(reference, segments, recovery=True)
            with (self.output/'recoveries.jsonl').open('a') as log:
                log.write(json.dumps({'source_sequence': saved[0]['sequence'],
                                      'last_visible_age_s': time.monotonic()-saved[1],
                                      'segments': segments})+'\n')
            return True
        return False
