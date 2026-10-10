"""Local geometric T pick/place policy with the original SO101 jaws and contact physics."""

import copy
import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import mujoco
import numpy as np

from lerobot_arm_control.control.control_defaults import CONTROL_CONFIG_SHA256, TRAJECTORY
from lerobot_arm_control.control.parking import smooth_fraction
from lerobot_arm_control.planning.push_t import JOINTS, build_scene, errors, pose, rotation
from lerobot_arm_control.robot.so101.model_metadata import MODEL
from lerobot_arm_control.transport.pick_t_protocol import STAGES, segment_digest
from lerobot_arm_control.visualization.tabletop_scene import scene_arms


class PickPlanner:
    def __init__(self, snapshot, output):
        self.output = Path(output)
        self.state, self.config, path = build_scene(snapshot, self.output)
        self.dim = self.state['models']['wood_t']
        if not self.dim.get('height_measured') or not .004 <= self.dim['height'] <= .015:
            raise ValueError('pick/place requires measured wood thickness in 4..15 mm')
        root = ET.parse(path)
        arms = scene_arms(self.state, self.config)
        wood = self.state['objects']['wood_t']
        wood_height = self.dim['height']/2+.00005
        root.find(".//body[@name='wood_t']").set('pos',
            ' '.join(map(str, [*wood['xy'], wood_height])))
        recorded = [q for arm in arms for q in arm['qpos']] + [*wood['xy'], wood_height,
            math.cos(wood['yaw']/2), 0, 0, math.sin(wood['yaw']/2)]
        root.find('keyframe/key').set('qpos', ' '.join(map(str, recorded)))
        # Other arms remain obstacles at their explicit preview poses in local physics.
        # The payload remains a free body, without an attachment constraint.
        for arm in arms[1:]:
            equality = root.find('equality')
            if equality is None:
                equality = ET.SubElement(root.getroot(), 'equality')
            for joint, value in zip(JOINTS, arm['qpos'], strict=True):
                ET.SubElement(equality, 'joint', joint1=arm['prefix']+joint,
                              polycoef=f'{value} 0 0 0 0', solref='.002 1')
        gripper = root.find(".//body[@name='gripper']")
        # The contact reference is derived from the existing fixed fingertip sphere.
        fixed = gripper.find("geom[@name='fixed_jaw_sph_tip1']")
        point = np.fromstring(fixed.get('pos'), sep=' ')
        point[0] += self.dim['stem_width']/2 + float(fixed.get('size'))
        ET.SubElement(gripper, 'site', name='pick_center', pos=' '.join(map(str, point)),
                      size='.0005', group='5')
        root.write(path)
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.qadr = np.array([self.model.joint(j).qposadr[0] for j in JOINTS])
        self.limits = np.array([self.model.joint(j).range for j in JOINTS])
        self.site = self.model.site('pick_center').id
        self.body = self.model.body('gripper').id
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.initial = self.data.qpos[self.qadr].copy()
        self.grasp_offset = np.array([0., self.dim['length']/2-self.dim['cap_depth']
                                      -self.dim['stem_width']])
        self.open_angle = self.aperture_angle(self.dim['stem_width']+.004)
        self.closed_angle = self.aperture_angle(self.dim['stem_width']-.001)
        self.height = self.dim['height']*2/3
        self.clearance = .05
        self.rng = np.random.default_rng(8)

    def aperture(self, angle):
        self.data.qpos[self.qadr[5]] = angle
        mujoco.mj_forward(self.model, self.data)
        fixed = self.data.geom('fixed_jaw_sph_tip1').xpos
        moving = self.data.geom('moving_jaw_sph_tip1').xpos
        axis = self.data.xmat[self.body].reshape(3, 3)[:, 0]
        return float((moving-fixed)@axis)

    def aperture_angle(self, distance):
        low, high = self.limits[5]
        high = min(high, .7)
        if not self.aperture(low) < distance < self.aperture(high):
            raise ValueError('T stem does not fit the original gripper aperture')
        for _ in range(40):
            mid = (low+high)/2
            if self.aperture(mid) < distance:
                low = mid
            else:
                high = mid
        return (low+high)/2

    def grasp_point(self, obj, height):
        return np.r_[np.array(obj['xy'])+rotation(obj['yaw'])@self.grasp_offset, height]

    def solve(self, xyz, yaw, jaw, seed, attempts=10):
        desired = np.eye(3)
        desired[:2, :2] = rotation(yaw)

        def residual(q):
            self.data.qpos[self.qadr] = np.r_[q, jaw]
            mujoco.mj_forward(self.model, self.data)
            matrix = self.data.xmat[self.body].reshape(3, 3)
            angle = cv2.Rodrigues(desired@matrix.T)[0].ravel()
            return np.r_[np.asarray(xyz)-self.data.site_xpos[self.site], .015*angle]

        for attempt in range(attempts):
            q = np.clip(seed[:5], self.limits[:5, 0]+.003, self.limits[:5, 1]-.003)
            if attempt:
                q = self.rng.uniform(self.limits[:5, 0]+.01, self.limits[:5, 1]-.01)
            for _ in range(180):
                error = residual(q)
                jac = np.column_stack([(residual(q+step)-residual(q-step))/(2e-5)
                                       for step in np.eye(5)*1e-5])
                delta = np.linalg.solve(jac.T@jac+np.eye(5)*1e-6, -jac.T@error)
                q = np.clip(q+np.clip(delta, -.12, .12), self.limits[:5, 0]+.003,
                            self.limits[:5, 1]-.003)
                if np.linalg.norm(delta) < 1e-6:
                    break
            error = residual(q)
            matrix = self.data.xmat[self.body].reshape(3, 3)
            tilt = math.acos(np.clip(matrix[2, 2], -1, 1))
            actual_yaw = math.atan2(matrix[1, 0], matrix[0, 0])
            turn = abs(math.atan2(math.sin(actual_yaw-yaw), math.cos(actual_yaw-yaw)))
            if np.linalg.norm(error[:3]) < .001 and tilt < .15 and turn < .06:
                return np.r_[q, jaw]
        raise ValueError('no downward grasp IK within position, orientation and joint bounds')

    def check_segments(self, segments, stage, object_pose):
        """Check original geometry and carried wood at every sampled joint configuration."""
        data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, data, 0)
        wood = self.model.body('wood_t').id
        wood_adr = self.model.joint('wood_free').qposadr[0]
        grasp_q = self.grasp_q
        data.qpos[self.qadr] = grasp_q
        mujoco.mj_forward(self.model, data)
        grasp_rotation = data.xmat[self.body].reshape(3, 3).copy()
        wood_xyz = np.r_[np.array(object_pose['xy']), self.dim['height']/2]
        offset = grasp_rotation.T@(wood_xyz-data.xpos[self.body])
        wood_rotation = np.eye(3)
        wood_rotation[:2, :2] = rotation(object_pose['yaw'])
        relative_rotation = grasp_rotation.T@wood_rotation
        carrying = stage in ('lift', 'transfer', 'lower')
        count = 0
        for segment in segments:
            start, end = map(np.asarray, (segment['start'], segment['end']))
            steps = max(21, math.ceil(np.max(np.abs(end-start))/.01)+1)
            for fraction in np.linspace(0, 1, steps):
                data.qpos[self.qadr] = start+(end-start)*fraction
                mujoco.mj_forward(self.model, data)
                if carrying:
                    matrix = data.xmat[self.body].reshape(3, 3)
                    data.qpos[wood_adr:wood_adr+3] = data.xpos[self.body]+matrix@offset
                    quaternion = np.empty(4)
                    mujoco.mju_mat2Quat(quaternion, (matrix@relative_rotation).ravel())
                    data.qpos[wood_adr+3:wood_adr+7] = quaternion
                else:
                    obj = self.target if stage in ('open', 'retreat') else object_pose
                    data.qpos[wood_adr:wood_adr+7] = [
                        *obj['xy'], self.dim['height']/2,
                        math.cos(obj['yaw']/2), 0, 0, math.sin(obj['yaw']/2)]
                mujoco.mj_forward(self.model, data)
                for contact in data.contact:
                    bodies = [self.model.geom_bodyid[g] for g in (contact.geom1, contact.geom2)]
                    if wood in bodies:
                        other = (contact.geom2 if bodies[0] == wood else contact.geom1)
                        name = self.model.geom(other).name or ''
                        intended = name.startswith(('fixed_jaw_', 'moving_jaw_')) or (
                            self.model.geom(other).group[0] == 4 and
                            self.model.body(self.model.geom_bodyid[other]).name in (
                                'gripper', 'moving_jaw_so101_v1'))
                        if (0 in bodies and contact.dist >= -.0005 or
                                intended and stage not in ('prepare', 'approach')):
                            continue
                    if contact.dist < -.0005:
                        names = [self.model.geom(g).name for g in (contact.geom1, contact.geom2)]
                        raise ValueError(f'{stage}: model collision {names}, '
                                         f'penetration {-contact.dist*1000:.2f} mm')
                count += 1
        return count

    def plan(self):
        if np.any(self.initial < self.limits[:, 0]) or np.any(self.initial > self.limits[:, 1]):
            raise ValueError('measured initial pose is outside CAD/encoder range intersection')
        wood = self.state['objects']['wood_t']
        self.target = copy.deepcopy(self.state['objects']['black_t'])
        for flip in (0., math.pi):
            try:
                source_yaw, target_yaw = wood['yaw']+flip, self.target['yaw']+flip
                opened = self.initial.copy()
                opened[5] = self.open_angle
                approach = self.solve(self.grasp_point(wood, self.height+self.clearance),
                                      source_yaw, self.open_angle, opened)
                descend = self.solve(self.grasp_point(wood, self.height), source_yaw,
                                     self.open_angle, approach)
                close = descend.copy()
                close[5] = self.closed_angle
                self.grasp_q = close.copy()
                lift = self.solve(self.grasp_point(wood, self.height+self.clearance),
                                  source_yaw, self.closed_angle, close)
                transfer = self.solve(self.grasp_point(self.target, self.height+self.clearance),
                                      target_yaw, self.closed_angle, lift)
                lower = self.solve(self.grasp_point(self.target, self.height), target_yaw,
                                   self.closed_angle, transfer)
                opened_at_goal = lower.copy()
                opened_at_goal[5] = self.open_angle
                retreat = self.solve(self.grasp_point(self.target, self.height+self.clearance),
                                     target_yaw, self.open_angle, opened_at_goal)
                waypoints = [opened, approach, descend, close, lift, transfer, lower,
                             opened_at_goal, retreat]
                previous, result = self.initial, []
                for stage, end in zip(STAGES, waypoints, strict=True):
                    speed = TRAJECTORY['entry_speed_rad_s'] if stage == 'approach' else .25
                    duration = max(TRAJECTORY['min_entry_seconds'],
                                   1.875*float(np.max(np.abs(end-previous)))/speed)
                    segment = {'start': previous.tolist(), 'end': end.tolist(),
                               'seconds': duration, 'speed_rad_s': speed}
                    count = self.check_segments([segment], stage, wood)
                    result.append({'stage': stage, 'segments': [segment],
                                   'collision_samples': count})
                    previous = end
                return {'stages': result, 'target': self.target, 'grasp_offset_m':
                        self.grasp_offset.tolist(), 'open_angle': self.open_angle,
                        'closed_angle': self.closed_angle, 'clearance_m': self.clearance,
                        'control_config_sha256': CONTROL_CONFIG_SHA256}
            except ValueError as exc:
                reason = str(exc)
        raise ValueError(reason)

    def simulate(self, plan):
        """No weld or scripted object movement: lift/transfer must emerge from jaw contact."""
        data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, data, 0)
        data.ctrl[:] = self.initial
        for _ in range(100):
            mujoco.mj_step(self.model, data)
        phases = []
        for stage in plan['stages']:
            for segment in stage['segments']:
                start, end = map(np.asarray, (segment['start'], segment['end']))
                count = math.ceil(segment['seconds']/self.model.opt.timestep)
                for tick in range(count):
                    data.ctrl[:] = start+smooth_fraction((tick+1)/count)*(end-start)
                    mujoco.mj_step(self.model, data)
                for _ in range(300):
                    data.ctrl[:] = end
                    mujoco.mj_step(self.model, data)
            height = float(data.body('wood_t').xpos[2])
            phases.append({'stage': stage['stage'], 'wood_center_height_m': height,
                           'wood_pose': pose(data).tolist()})
            if stage['stage'] in ('lift', 'transfer') and height < .02:
                return {'passed': False, 'reason': 'contact grasp did not retain lifted wood',
                        'phases': phases}
        distance, yaw = errors(pose(data), np.r_[self.target['xy'], self.target['yaw']])
        return {'passed': distance <= .005 and yaw <= math.radians(8), 'phases': phases,
                'final_center_error_m': distance, 'final_yaw_error_rad': yaw,
                'scope': 'local contact simulation; no physical success claim'}

    def validation(self, segments):
        return {'checked': True, 'segments_sha256': segment_digest(segments),
                'control_config_sha256': CONTROL_CONFIG_SHA256,
                'model_sha256': hashlib.sha256(MODEL.read_bytes()).hexdigest()}
