"""Offline SO101 Push-T with contact rollouts and receding-horizon feedback.

Simulation is local. Optional --capture acquires one remote read-only observation first.
"""

import argparse
import copy
import json
import math
import shutil
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from lerobot_arm_control.control.control_defaults import SIMULATION_ACTUATOR
from lerobot_arm_control.core.paths import resource_path
from lerobot_arm_control.core.runtime_config import add_remote_arguments
from lerobot_arm_control.robot.so101.joints import JOINTS
from lerobot_arm_control.visualization.tabletop_scene import ROOT, scene_arms, scene_xml

CONTACT_CLEARANCE = .007
JAW_CLOSED = -.16
CONTACT_HEIGHT = .0035


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotation(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])


def outline(dim):
    w, length, cap, stem = (dim[k] for k in ('width', 'length', 'cap_depth', 'stem_width'))
    return np.array([[-w/2, length/2], [w/2, length/2], [w/2, length/2-cap],
                     [stem/2, length/2-cap], [stem/2, -length/2], [-stem/2, -length/2],
                     [-stem/2, length/2-cap], [-w/2, length/2-cap]])


def pose(data):
    b = data.body('wood_t')
    mat = b.xmat.reshape(3, 3)
    return np.r_[b.xpos[:2], math.atan2(mat[1, 0], mat[0, 0])]


def errors(current, target):
    return float(np.linalg.norm(current[:2]-target[:2])), abs(wrap(current[2]-target[2]))


def objective(current, target):
    distance, yaw = errors(current, target)
    return distance**2 + (.024*yaw)**2


def freeze_additional_arms(world, state, config):
    """Keep observed bystander robots fixed while only the follower executes pushes."""
    arms = [arm for arm in scene_arms(state, config) if arm['prefix']]
    if not arms:
        return
    for arm in arms:
        from lerobot_arm_control.robot.so101.model_metadata import display_model

        model = mujoco.MjModel.from_xml_path(str(display_model(
            arm['config'].get('simulation', {}).get('end_effector', 'gripper'))))
        data = mujoco.MjData(model)
        data.qpos[:] = arm['qpos']
        mujoco.mj_forward(model, data)
        root = world.find(f"body[@name='{arm['root']}']")
        root.attrib.pop('mocap')
        for body in root.iter('body'):
            try:
                index = model.body(body.get('name').removeprefix(arm['prefix'])).id
            except KeyError:
                continue
            parent = model.body_parentid[index]
            rotation = data.xmat[parent].reshape(3, 3).T
            position = rotation@(data.xpos[index]-data.xpos[parent])
            quaternion = np.empty(4)
            mujoco.mju_mat2Quat(quaternion, (rotation@data.xmat[index].reshape(3, 3)).ravel())
            body.set('pos', ' '.join(map(str, position)))
            body.set('quat', ' '.join(map(str, quaternion)))
            for joint in body.findall('joint'):
                body.remove(joint)


def build_scene(snapshot, output, *, freeze_bystanders=True):
    source = Path(snapshot)
    state = json.loads((source/'state.json').read_text())
    config = json.loads((source/'calibration.json').read_text())['config']
    if not all(name in state['objects'] for name in ('black_t', 'wood_t')):
        raise ValueError('Measured poses for both T shapes are required')
    if 'position' not in state['robot']['registration']:
        raise ValueError('Measured robot base pose is required')
    output.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source/'appearance', output/'appearance', dirs_exist_ok=True)
    meshes = source/'robot_assets'
    if not meshes.is_dir():
        meshes = resource_path('assets/so101/assets')
    shutil.copytree(meshes, output/'robot_assets', dirs_exist_ok=True)
    shutil.copyfile(source/'marker.png', output/'marker.png')
    root = ET.fromstring(scene_xml(state, config, str((output/'robot_assets').resolve())))
    root.set('model', 'Measured tabletop - offline SO101 Push-T')
    root.find('option').attrib.update(gravity='0 0 -9.81', timestep='.002',
                                     integrator='implicitfast', cone='elliptic', iterations='30')
    root.remove(root.find('keyframe'))
    world = root.find('worldbody')
    if freeze_bystanders:
        freeze_additional_arms(world, state, config)
    world.find("body[@name='robot_root']").attrib.pop('mocap')
    black = world.find("body[@name='black_t']")
    black.attrib.pop('mocap')
    # A print has no collision or raised physical silhouette.
    black.set('pos', f"{state['objects']['black_t']['xy'][0]} "
                    f"{state['objects']['black_t']['xy'][1]} .00016")
    for geom in black.findall('geom'):
        size = geom.get('size').split()
        size[2] = '.00008'
        geom.set('size', ' '.join(size))
    wood = world.find("body[@name='wood_t']")
    wood.attrib.pop('mocap')
    wood.set('pos', ' '.join(map(str, [*state['objects']['wood_t']['xy'], .00305])))
    ET.SubElement(wood, 'freejoint', name='wood_free')
    for geom in wood.findall('geom'):
        geom.attrib.update(contype='1', conaffinity='7', density='700', condim='3',
                           friction='.4 .001 .0001', solref='.006 1', solimp='.95 .99 .001')
    world.find("geom[@name='tabletop_workspace']").attrib.update(
        contype='1', conaffinity='7', friction='.4 .001 .0001', condim='3')
    for geom in world.iter('geom'):
        if geom.get('class', '').startswith('collision'):
            geom.attrib.update(contype='4', conaffinity='1')
    site = world.find(".//site[@name='gripperframe']")
    parent = next(body for body in world.iter('body') if site in list(body))
    # Reference lies on the original fixed jaw's terminal capsule; no added tool.
    contact = parent.find("geom[@name='fixed_jaw_box3']")
    ET.SubElement(parent, 'site', name='push_contact', pos=contact.get('pos'), size='.0005',
                  rgba='0 0 0 0', group='3')
    for body in world.iter('body'):
        for index, geom in enumerate(body.findall('geom')):
            if geom.get('class', '').startswith('collision') and not geom.get('name'):
                geom.set('name', f"{body.get('name')}_collision_{index}")
    actuator = ET.SubElement(root, 'actuator')
    calibration = json.loads((source/'joint_calibration.json').read_text())
    fits = json.loads((source/'calibration.json').read_text()).get('robot', {}).get('fits', {})
    for joint in JOINTS:
        element = root.find(f".//joint[@name='{joint}']")
        low, high = np.fromstring(element.get('range'), sep=' ')
        points = calibration['points'][joint]
        zero, thirty = points['0']['raw'], points['30']['raw']
        scale = math.radians(30)/(thirty-zero)
        if joint in fits:
            zero, scale = fits[joint]['zero_raw'], fits[joint]['radians_per_tick']
        raw_limits = calibration['fingerprint']['raw_limits'][joint]
        measured = sorted((raw-zero)*scale for raw in raw_limits)
        limits = f'{max(low, measured[0])} {min(high, measured[1])}'
        element.set('range', limits)
        ET.SubElement(actuator, 'position', name='sim_'+joint, joint=joint,
                      kp=str(SIMULATION_ACTUATOR['kp']), kv=str(SIMULATION_ACTUATOR['kv']),
                      ctrlrange=limits, forcerange='-2.94 2.94')
    wood_pose = state['objects']['wood_t']
    robot_qpos = (state['robot']['qpos'] if freeze_bystanders else
                  [q for arm in scene_arms(state, config) for q in arm['qpos']])
    recorded = [*robot_qpos, *wood_pose['xy'], .00305,
                math.cos(wood_pose['yaw']/2), 0, 0, math.sin(wood_pose['yaw']/2)]
    ET.SubElement(ET.SubElement(root, 'keyframe'), 'key', name='measured_start',
                  qpos=' '.join(map(str, recorded)),
                  ctrl=' '.join(map(str, state['robot']['qpos'])))
    path = output/'scene.xml'
    ET.indent(root)
    path.write_text(ET.tostring(root, encoding='unicode'))
    return state, config, path


class Arm:
    """Damped IK with joint limits and desk clearance for the original gripper contact."""
    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        self.site = model.site('push_contact').id
        self.frame = model.site('gripperframe').id
        self.qadr = np.array([model.joint(name).qposadr[0] for name in JOINTS])
        self.dadr = np.array([model.joint(name).dofadr[0] for name in JOINTS])
        self.limits = np.array([model.joint(name).range for name in JOINTS])
        self.jp = np.zeros((3, model.nv))
        self.jr = np.zeros((3, model.nv))
        self.rng = np.random.default_rng(42)

    def solve(self, xyz, seed, *, attempts=1, clearance=True):
        m, d = self.model, self.data
        best = None
        for attempt in range(attempts):
            q = np.array(seed).copy() if attempt == 0 else self.rng.uniform(
                self.limits[:, 0]+.015, self.limits[:, 1]-.015)
            q[4], q[5] = 0, JAW_CLOSED
            d.qpos[self.qadr] = q
            for _ in range(100):
                mujoco.mj_forward(m, d)
                error = np.asarray(xyz)-d.site_xpos[self.site]
                axis = d.site_xmat[self.frame].reshape(3, 3)[:, 0]
                # Position is strict; downward finger posture is a soft preference.
                residual = np.r_[error, .0005*(np.array([0, 0, -1])-axis)]
                mujoco.mj_jacSite(m, d, self.jp, self.jr, self.site)
                skew = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                                 [-axis[1], axis[0], 0]])
                jac = np.vstack([self.jp[:, self.dadr[:4]],
                                 -.0005*skew@self.jr[:, self.dadr[:4]]])
                dq = jac.T@np.linalg.solve(jac@jac.T+np.eye(6)*2e-7, residual)
                d.qpos[self.qadr[:4]] = np.clip(d.qpos[self.qadr[:4]]+np.clip(dq, -.12, .12),
                                               self.limits[:4, 0]+.015,
                                               self.limits[:4, 1]-.015)
                if np.linalg.norm(error) < .00025:
                    break
            mujoco.mj_forward(m, d)
            error = np.linalg.norm(np.asarray(xyz)-d.site_xpos[self.site])
            collision = clearance and any(
                c.dist < -.0005 and m.geom_bodyid[c.geom1] != m.body('wood_t').id
                and m.geom_bodyid[c.geom2] != m.body('wood_t').id
                for c in d.contact)
            if error < .001 and not collision:
                result = d.qpos[self.qadr].copy()
                delta = np.linalg.norm(result-np.asarray(seed))
                if best is None or delta < best[0]:
                    best = delta, result
                if attempt == 0:
                    break
        return best[1] if best else None

    def path_clear(self, start, finish, *, avoid_wood=False):
        spacing = .015 if avoid_wood else .04
        initial_contacts = {}
        wood = self.model.body('wood_t').id
        for fraction in np.linspace(0, 1, max(3, int(np.max(np.abs(finish-start))/spacing)+1)):
            self.data.qpos[self.qadr] = start+(finish-start)*fraction
            mujoco.mj_forward(self.model, self.data)
            for contact in self.data.contact:
                pair = tuple(sorted((contact.geom1, contact.geom2)))
                is_wood = any(self.model.geom_bodyid[g] == wood for g in pair)
                # Wood/table compliance is not a robot transfer collision.
                if is_wood and (not avoid_wood or any(
                        self.model.geom_bodyid[g] == 0 for g in pair)):
                    continue
                if is_wood and fraction == 0:
                    initial_contacts[pair] = contact.dist
                # A jaw already pressing into compliant contact may depart from it,
                # but transfers must not deepen overlap or finish inside the wood.
                threshold = min(-.0005, initial_contacts.get(pair, 0)-.00025)
                if contact.dist < (threshold if is_wood and fraction < 1 else -.0005):
                    return False
        return True


@dataclass
class Push:
    start: np.ndarray
    direction: np.ndarray
    distance: float
    score: float = math.inf
    joints: list | None = None
    approach_count: int = 0
    transition: str = 'low transfer'


class PushSimulation:
    @staticmethod
    def interpolate(fraction):
        return fraction*fraction*(3-2*fraction)

    def __init__(self, snapshot=ROOT/'data/push_t/measured', output=ROOT/'data/push_t/run'):
        self.state, self.config, path = build_scene(snapshot, Path(output))
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.arm = Arm(self.model)
        self.qadr = self.arm.qadr
        self.data.qpos[self.qadr] = self.state['robot']['qpos']
        self.data.ctrl[:] = self.data.qpos[self.qadr]
        self.target = np.r_[self.state['objects']['black_t']['xy'],
                            self.state['objects']['black_t']['yaw']]
        self.dim = self.state['models']['wood_t']
        self.polygon = outline(self.dim)
        self.initial = np.r_[self.state['objects']['wood_t']['xy'],
                             self.state['objects']['wood_t']['yaw']]
        self.wood_joint = self.model.joint('wood_free')
        self.wood_qadr = self.wood_joint.qposadr[0]
        self.bounds = self.state['appearance']['bounds']
        self.trace = []
        self.pushes = 0
        self.phase = 'ready'
        self.render_callback = None
        self.record_callback = None
        self.jaw_geoms = self._jaw_geoms()
        self.contact_counts = {}
        self._rollout_model(path)
        self.reset(self.initial)

    def _jaw_geoms(self):
        gripper = self.model.body('gripper').id
        bodies = set()
        for body in range(self.model.nbody):
            parent = body
            while parent > 0 and parent != gripper:
                parent = self.model.body_parentid[parent]
            if parent == gripper:
                bodies.add(body)
        return {i for i in range(self.model.ngeom)
                if self.model.geom_bodyid[i] in bodies and self.model.geom_contype[i]}

    def _rollout_model(self, path):
        # Prediction and execution use the identical joint-driven original SO101 CAD.
        rollout_path = path.with_name('rollout.xml')
        shutil.copyfile(path, rollout_path)
        self.roll_model = mujoco.MjModel.from_xml_path(str(rollout_path))
        self.roll_data = mujoco.MjData(self.roll_model)

    def reset(self, initial):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr] = self.state['robot']['qpos']
        self.data.ctrl[:] = self.data.qpos[self.qadr]
        self.data.qpos[self.wood_qadr:self.wood_qadr+7] = [
            *initial[:2], .00305, math.cos(initial[2]/2), 0, 0, math.sin(initial[2]/2)]
        mujoco.mj_forward(self.model, self.data)
        self.pushes, self.trace, self.phase = 0, [], 'ready'
        self.contact_counts = {}
        self.simulation_controls = {}
        controller = self.position_controller(self.model, self.data)
        for _ in range(80):
            controller.step(self.state['robot']['qpos'])

    def position_controller(self, model, data):
        from lerobot_arm_control.control.simulation_control import SimulationControl

        if not hasattr(self, 'simulation_controls'):
            self.simulation_controls = {}
        key = id(data)
        if key not in self.simulation_controls:
            self.simulation_controls[key] = SimulationControl(model, data)
        return self.simulation_controls[key]

    def valid_pose(self, value):
        points = self.polygon@rotation(value[2]).T+value[:2]
        xmin, xmax, ymin, ymax = self.bounds
        if not np.all((points[:, 0] > xmin+.015) & (points[:, 0] < xmax-.015)
                      & (points[:, 1] > ymin+.015) & (points[:, 1] < ymax-.015)):
            return False
        base = self.state['robot']['registration']['position']
        return np.linalg.norm(value[:2]-base[:2]) > .085

    def candidates(self, current):
        distance, angle = errors(current, self.target)
        stroke = np.clip(max(distance*.35, angle*.012), .0004, .014)
        rot = rotation(current[2])
        polygon = self.polygon@rot.T+current[:2]
        candidates = []
        wood = self.model.body('wood_t').id
        touching = any(
            self.model.geom_bodyid[c.geom1] == wood and c.geom2 in self.jaw_geoms
            or self.model.geom_bodyid[c.geom2] == wood and c.geom1 in self.jaw_geoms
            for c in self.data.contact)
        site = self.data.site('push_contact').xpos
        if touching and abs(site[2]-CONTACT_HEIGHT) < .002:
            inward = current[:2]-site[:2]
            if np.linalg.norm(inward) > 1e-6:
                inward = inward/np.linalg.norm(inward)
                for turn in (-.65, 0, .65):
                    candidates.append(Push(site[:2].copy(), rotation(turn)@inward, float(stroke),
                                           transition='contact continuation'))
            direction = self.target[:2]-current[:2]
            if np.linalg.norm(direction) > 1e-6 and np.linalg.norm(inward) > 1e-6:
                direction /= np.linalg.norm(direction)
                if direction@inward > .15:
                    candidates.append(Push(site[:2].copy(), direction, float(stroke),
                                           transition='contact continuation'))
        for i, a in enumerate(polygon):
            b = polygon[(i+1) % len(polygon)]
            tangent = (b-a)/np.linalg.norm(b-a)
            normal = np.array([-tangent[1], tangent[0]])
            for f in (.22, .5, .78):
                contact = a+(b-a)*f
                start = contact+normal*CONTACT_CLEARANCE
                for turn in (-.65, 0, .65):
                    direction = rotation(turn)@(-normal)
                    candidates.append(Push(start, direction, float(stroke+CONTACT_CLEARANCE)))
        return candidates

    def trajectory(self, action, *, lift=False):
        seed = self.data.qpos[self.qadr].copy()
        start = np.r_[action.start, CONTACT_HEIGHT]
        finish = start+np.r_[action.direction, 0]*action.distance
        if action.transition == 'contact continuation' and not lift:
            approach = []
        elif lift:
            site = self.data.site('push_contact').xpos
            height = max(site[2], CONTACT_HEIGHT+.035)
            approach = [np.r_[site[:2], height], start+[0, 0, .035], start]
        else:
            approach = [start]
        options = [(approach, 'lift to change side' if lift else action.transition)]
        if lift and site[2] < CONTACT_HEIGHT+.002:
            outward = site[:2]-self.data.body('wood_t').xipos[:2]
            if np.linalg.norm(outward) > 1e-6:
                outward /= np.linalg.norm(outward)
                releases = [np.r_[site[:2]+rotation(turn)@outward*distance, CONTACT_HEIGHT]
                            for distance in (.008, .016) for turn in (0, -.6, .6)]
                # Try horizontal departure first. A trapped jaw may not lift vertically.
                options = [([release, start], 'low transfer via release') for release in releases]
                options.append((approach, 'lift to change side'))
                options.extend(([release, release+[0, 0, .035], start+[0, 0, .035], start],
                                'lift to change side') for release in releases)
        pushing = [start+(finish-start)*f for f in np.linspace(.1, 1, 10)]
        for approach, transition in options:
            joints = []
            points = approach+pushing
            for index, xyz in enumerate(points):
                previous = seed if not joints else joints[-1]
                q = self.arm.solve(xyz, previous, attempts=8 if not joints else 1)
                if q is None or not self.arm.path_clear(previous, q,
                                                       avoid_wood=index < len(approach)):
                    break
                joints.append(q)
            if len(joints) == len(points):
                action.joints = joints
                action.approach_count = len(approach)
                action.transition = transition
                return True
        return False

    def predict(self, action, friction=.4):
        m, d = self.roll_model, self.roll_data
        mujoco.mj_copyData(d, m, self.data)
        self.position_controller(m, d).copy_state_from(self.position_controller(
            self.model, self.data))
        m.geom_friction[m.geom('tabletop_workspace').id, 0] = friction
        for name in ('wood_t_cap', 'wood_t_stem'):
            m.geom_friction[m.geom(name).id, 0] = friction
        self.play_action(action, m, d)
        result = pose(d)
        if (not self.valid_pose(result) or abs(d.body('wood_t').xmat[8]-1) > .02
                or abs(d.site('push_contact').xpos[2]-CONTACT_HEIGHT) >= .001):
            return math.inf
        return objective(result, self.target)

    def plan(self):
        current = pose(self.data)
        # Low transfers check all original jaw geometry against the current wood pose.
        mujoco.mj_copyData(self.arm.data, self.model, self.data)
        actions = self.candidates(current)
        # A wrench estimate only orders candidates; selection uses original CAD contact rollouts.
        center_of_mass = self.data.body('wood_t').xipos[:2]
        def estimate(action):
            lever = action.start-center_of_mass
            torque = lever[0]*action.direction[1]-lever[1]*action.direction[0]
            predicted = current+np.r_[action.direction*action.distance,
                                      float(torque)*action.distance/.026**2]
            return objective(predicted, self.target)
        actions.sort(key=estimate)
        feasible = []
        for lift in (False, True):
            for action in actions:
                if lift and action.transition == 'contact continuation':
                    continue
                if not self.trajectory(action, lift=lift):
                    continue
                action.score = self.predict(action)
                if math.isfinite(action.score):
                    feasible.append(action)
                if len(feasible) >= 20:
                    break
            if lift or (feasible and min(a.score for a in feasible)
                        < objective(current, self.target)*.999):
                break
            feasible = []
        if not feasible:
            self.phase = 'unreachable / collision'
            return None
        feasible.sort(key=lambda a: a.score)
        for action in feasible[:4]:
            action.score = max(action.score, self.predict(action, friction=.22),
                               self.predict(action, friction=.65))
        return min(feasible[:4], key=lambda a: a.score)

    def step(self, target, duration, model=None, data=None):
        model = self.model if model is None else model
        data = self.data if data is None else data
        controller = self.position_controller(model, data)
        initial = np.asarray(controller.reference)
        steps = max(1, int(duration/model.opt.timestep))
        for index in range(steps):
            f = (index+1)/steps
            controller.step(initial+(target-initial)*self.interpolate(f))
            if data is self.data:
                wood = model.body('wood_t').id
                for contact in data.contact:
                    pair = (contact.geom1, contact.geom2)
                    if any(model.geom_bodyid[g] == wood for g in pair):
                        for geom in set(pair) & self.jaw_geoms:
                            name = model.geom(geom).name
                            self.contact_counts[name] = self.contact_counts.get(name, 0)+1
                if self.render_callback and index % 8 == 0:
                    self.render_callback()
                if self.record_callback:
                    self.record_callback(self)

    def move(self, target, model=None, data=None):
        model = self.model if model is None else model
        data = self.data if data is None else data
        duration = max(.15, np.max(np.abs(target-self.position_controller(
            model, data).reference))/.55)
        self.step(target, duration, model, data)
        self.step(target, .18, model, data)

    def play_action(self, action, model, data):
        for q in action.joints[:action.approach_count]:
            self.move(q, model, data)
        pushing = action.joints[action.approach_count:]
        for q in pushing:
            self.step(q, action.distance/len(pushing)/.015, model, data)
        # Hold at the pushing height for feedback; chunk boundaries do not retract the jaw.
        target = np.array(self.position_controller(model, data).reference)
        self.step(target, .12, model, data)
        # A retuned controller may still be settling at the old fixed 120 ms boundary.
        # Observe the settled contact pose before planning the next push, with a bound
        # so an obstructed jaw cannot stall either execution or a candidate rollout.
        for _ in range(12):
            if (abs(data.site('push_contact').xpos[2]-CONTACT_HEIGHT) < .00075
                    and np.max(np.abs(data.qvel[self.arm.dadr])) < .02):
                break
            self.step(target, .04, model, data)

    def execute(self, action):
        self.phase = 'original gripper pushing'
        self.play_action(action, self.model, self.data)
        self.pushes += 1
        now = pose(self.data)
        distance, yaw = errors(now, self.target)
        self.trace.append({'push': self.pushes, 'pose': now.tolist(),
                           'position_error_m': distance, 'yaw_error_rad': yaw,
                           'action': {'start': action.start.tolist(),
                                      'direction': action.direction.tolist(),
                                      'distance': action.distance,
                                      'transition': action.transition,
                                      'approach_waypoints': action.approach_count},
                           'gripper_height_m': float(self.data.site('push_contact').xpos[2])})
        return True

    @property
    def success(self):
        distance, angle = errors(pose(self.data), self.target)
        return distance < .0015 and angle < math.radians(3)

    def solve(self, limit=100):
        started = time.monotonic()
        while not self.success and self.pushes < limit:
            self.phase = 'planning'
            action = self.plan()
            if action is None or not self.execute(action):
                break
            if self.pushes % 5 == 0:
                print(json.dumps({'push': self.pushes,
                                  'errors': errors(pose(self.data), self.target),
                                  'wall_s': time.monotonic()-started}), flush=True)
        self.phase = 'success' if self.success else 'failed: '+self.phase
        return {**self.result(), 'wall_seconds': time.monotonic()-started}

    def result(self):
        return {'success': self.success, 'pushes': self.pushes,
                'final_pose': pose(self.data).tolist(),
                'position_error_m': errors(pose(self.data), self.target)[0],
                'yaw_error_rad': errors(pose(self.data), self.target)[1],
                'phase': self.phase, 'trace': self.trace,
                'target_pose': self.target.tolist(), 'measured_initial_pose': self.initial.tolist(),
                'simulation_only': True,
                'base_calibrated': self.state['robot']['registration'].get('calibrated', False),
                'initialization': self.state.get('initialization',
                                                 {'kind': 'saved_observation'}),
                'trajectory': 'trajectory.jsonl' if self.record_callback else None,
                'pusher': 'original_so101_closed_gripper', 'jaw_contacts': self.contact_counts,
                'chunk_end_retraction': False,
                'lift_transfers': sum(t['action']['transition'] == 'lift to change side'
                                      for t in self.trace),
                'contact_continuations': sum(t['action']['transition'] == 'contact continuation'
                                            for t in self.trace),
                'gripper_angle_rad': JAW_CLOSED,
                'qpos': self.data.qpos.tolist(), 'simulation_time_s': self.data.time}


@dataclass
class PlannedPush:
    action: Push | None
    state: np.ndarray
    phase: str
    prefetched: bool = False

    def matches(self, model, data):
        current = np.empty_like(self.state)
        mujoco.mj_getState(model, data, current, mujoco.mjtState.mjSTATE_INTEGRATION)
        return np.allclose(current, self.state, rtol=0, atol=1e-7)


class PushPlanner:
    """Plan ahead on independent physics; validate against actual state before use."""
    def __init__(self, sim):
        self.sim = copy.copy(sim)
        self.sim.simulation_controls = {}
        self.sim.model = copy.copy(sim.model)
        self.sim.data = mujoco.MjData(self.sim.model)
        self.sim.arm = Arm(self.sim.model)
        self.sim.roll_model = copy.copy(sim.model)
        self.sim.roll_data = mujoco.MjData(self.sim.roll_model)
        self.sim.render_callback = None
        self.sim.record_callback = None
        self.sim.contact_counts = {}
        self.sim.trace = []

    def capture(self, sim):
        # Called on the UI thread only when the previous planning job has finished.
        mujoco.mj_copyData(self.sim.data, self.sim.model, sim.data)
        self.sim.position_controller(self.sim.model, self.sim.data).copy_state_from(
            sim.position_controller(sim.model, sim.data))

    def plan(self, after=None):
        sim = self.sim
        if after is not None:
            sim.play_action(after, sim.model, sim.data)
        state = np.empty(mujoco.mj_stateSize(sim.model, mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_getState(sim.model, sim.data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
        action = None if sim.success else sim.plan()
        return PlannedPush(action, state, sim.phase, after is not None)


def run_viewer(sim, args):
    import concurrent.futures

    import glfw

    from lerobot_arm_control.visualization.control_viewer import ControlViewer

    running = True
    pending_reset = None
    completed = False
    rng = np.random.default_rng(29)
    future = None
    pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    planner = PushPlanner(sim)
    prefetch_hits = stale_plans = 0
    frame_times = []

    def key(keycode):
        nonlocal running, pending_reset
        if keycode == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(viewer.window, True)
        elif keycode == glfw.KEY_SPACE:
            running = not running
        elif keycode == glfw.KEY_R:
            pending_reset = sim.initial.copy()
        elif keycode == glfw.KEY_N:
            pending_reset = sim.target+np.r_[rng.uniform(-.065, .065, 2), 0]
            pending_reset[2] = rng.uniform(-math.pi, math.pi)
        elif keycode in (glfw.KEY_LEFT, glfw.KEY_RIGHT, glfw.KEY_UP, glfw.KEY_DOWN,
                          glfw.KEY_Q, glfw.KEY_E):
            pending_reset = pose(sim.data).copy()
            axis = 0 if keycode in (glfw.KEY_LEFT, glfw.KEY_RIGHT) else 1
            if keycode in (glfw.KEY_Q, glfw.KEY_E):
                pending_reset[2] += .3 if keycode == glfw.KEY_Q else -.3
            else:
                pending_reset[axis] += .015 if keycode in (glfw.KEY_RIGHT, glfw.KEY_UP) else -.015
            running = False

    viewer = ControlViewer(sim.model, sim.data, key, 'SO101 Push-T - OFFLINE simulation',
                           hold_on_focus_loss=False)
    viewer.cam.lookat[:] = [*sim.target[:2], .045]
    viewer.cam.distance = .75
    observer = sim.state['camera_position']
    viewer.cam.azimuth = 180+math.degrees(math.atan2(observer[1]-sim.target[1],
                                                   observer[0]-sim.target[0]))
    viewer.cam.elevation = -58
    last_render = 0
    screenshot_saved = False
    wall_start, sim_start = time.monotonic(), sim.data.time

    def render(*, force=False):
        nonlocal last_render, wall_start, sim_start
        if not viewer.is_running():
            return
        if not force and not args.realtime and time.monotonic()-last_render < 1/60:
            return
        distance, yaw = errors(pose(sim.data), sim.target)
        viewer.set_texts([(0, 0,
            f"OFFLINE SO101 Push-T\n"
            f"State: {sim.phase}\n"
            f"Controller: {'running' if running else 'paused'}\n"
            f"Playback: {'real time' if args.realtime else 'fast'}\n"
            f"Next push: {'computing in background' if future and not future.done() else 'ready'}\n"
            f"Pushes: {sim.pushes}\n"
            f"Position error: {distance*1000:.2f} mm\n"
            f"Angle error: {math.degrees(yaw):.2f} deg\n"
            f"Goal: <1.5 mm and <3 deg\n\n"
            f"Target xy: {sim.target[0]:.4f}, {sim.target[1]:.4f} m\n"
            f"Target angle: {math.degrees(sim.target[2]):.1f} deg\n"
            f"T: 48 x 60 x 6 mm\n"
            f"SO101 base: measured visual estimate\n"
            f"Pusher: original closed SO101 jaws\n"
            f"Physics: gravity / friction / contact\n"
            f"Joint limits: CAD + saved calibration\n\n"
            f"Space: run / pause\nR: restore measured start\nN: random start\n"
            f"Arrows: move initial T (paused)\nQ / E: rotate initial T (paused)\n"
            f"Mouse: orbit / zoom | Esc: close")])
        viewer.sync()
        last_render = time.monotonic()
        if not completed:
            frame_times.append(last_render)
        if args.realtime:
            deadline = wall_start+(sim.data.time-sim_start)
            time.sleep(max(0, min(.04, deadline-time.monotonic())))

    sim.render_callback = render
    try:
        while viewer.is_running():
            if pending_reset is not None and (future is None or future.done()):
                value, pending_reset = pending_reset, None
                if sim.valid_pose(value):
                    sim.reset(value)
                    completed = False
                    future = None
                    frame_times.clear()
                    prefetch_hits = stale_plans = 0
                    screenshot_saved = False
                    wall_start, sim_start = time.monotonic(), sim.data.time
            if sim.success:
                sim.phase = 'SUCCESS - aligned and settled'
                if not completed:
                    result = sim.result()
                    intervals = np.diff(frame_times)
                    result['playback'] = {
                        'mode': 'realtime' if args.realtime else 'fast',
                        'frames': len(frame_times),
                        'mean_render_fps': float(1/np.mean(intervals)) if len(intervals) else 0,
                        'max_frame_gap_ms': float(np.max(intervals)*1000) if len(intervals) else 0,
                        'prefetched_plans_used': prefetch_hits,
                        'stale_plans_rejected': stale_plans,
                    }
                    (args.output/'result.json').write_text(json.dumps(result, indent=2))
                    print(json.dumps({k: v for k, v in result.items() if k != 'trace'}), flush=True)
                    completed = True
                if args.screenshot and not screenshot_saved:
                    args.screenshot.parent.mkdir(parents=True, exist_ok=True)
                    render(force=True)
                    viewer.sync(capture_path=args.screenshot)
                    screenshot_saved = True
            elif running and pending_reset is None:
                if sim.pushes >= args.limit:
                    sim.phase = 'iteration budget exhausted'
                    running = False
                elif future is None:
                    sim.phase = 'planning contact + IK'
                    planner.capture(sim)
                    future = pool.submit(planner.plan)
                elif future.done():
                    planned = future.result()
                    future = None
                    if not planned.matches(sim.model, sim.data):
                        stale_plans += 1
                        sim.phase = 'state changed - replanning'
                    elif planned.action is None:
                        sim.phase = planned.phase
                        running = False
                    else:
                        if planned.prefetched:
                            prefetch_hits += 1
                        # Forecast the end of this stroke while the main thread plays it.
                        # Actual full integration state must match before any forecast is used.
                        planner.capture(sim)
                        if sim.pushes+1 < args.limit:
                            future = pool.submit(planner.plan, planned.action)
                        wall_start, sim_start = time.monotonic(), sim.data.time
                        sim.execute(planned.action)
                        print(json.dumps({'push': sim.pushes,
                                          'errors': errors(pose(sim.data), sim.target)}),
                              flush=True)
            if time.monotonic()-last_render > .016:
                render()
            time.sleep(.004)
    finally:
        sim.render_callback = None
        pool.shutdown(wait=True, cancel_futures=True)
        viewer.close()


def benchmark(sim, count, seed):
    rng = np.random.default_rng(seed)
    results = []
    for index in range(count):
        initial = sim.initial.copy() if index == 0 else sim.target.copy()
        if index:
            initial[:2] += rng.uniform([-.06, -.10], [.075, .08])
            initial[2] = rng.uniform(-math.pi, math.pi)
        friction = rng.uniform(.2, .65)
        density = rng.uniform(450, 950)
        mass_factor = density/700
        wood = sim.model.body('wood_t').id
        old_mass = sim.model.body_mass[wood]
        old_inertia = sim.model.body_inertia[wood].copy()
        sim.model.body_mass[wood] *= mass_factor
        sim.model.body_inertia[wood] *= mass_factor
        mujoco.mj_setConst(sim.model, sim.data)
        for name in ('tabletop_workspace', 'wood_t_cap', 'wood_t_stem'):
            sim.model.geom_friction[sim.model.geom(name).id, 0] = friction
        sim.reset(initial)
        result = sim.solve(100)
        result.pop('trace')
        result.update(case=index, initial=initial.tolist(), friction=float(friction),
                      density_kg_m3=float(density))
        results.append(result)
        sim.model.body_mass[wood] = old_mass
        sim.model.body_inertia[wood] = old_inertia
        print(json.dumps({'benchmark_case': result}), flush=True)
    return {'seed': seed, 'cases': results, 'successes': sum(r['success'] for r in results),
            'count': count, 'position_tolerance_m': .0015, 'yaw_tolerance_deg': 3,
            'scope': 'measured start + sampled collision-free SO101 task region',
            'friction_range': [.2, .65], 'density_range_kg_m3': [450, 950]}


class RolloutRecorder:
    """Record executed physics at 62.5 Hz; candidate predictions are never recorded."""
    def __init__(self, path):
        self.file = path.open('w')
        self.last_time = -math.inf
        self.episode = 0

    def record(self, sim):
        if sim.data.time < self.last_time:
            self.episode += 1
            self.last_time = -math.inf
        if sim.data.time-self.last_time < .016-1e-9:
            return
        self.file.write(json.dumps({'episode': self.episode, 'time_s': sim.data.time,
                                    'push': sim.pushes, 'phase': sim.phase,
                                    'qpos': sim.data.qpos.tolist(),
                                    'qvel': sim.data.qvel.tolist(),
                                    'ctrl': sim.data.ctrl.tolist(),
                                    'wood_pose': pose(sim.data).tolist()}, allow_nan=False)+'\n')
        self.file.flush()
        self.last_time = sim.data.time

    def close(self):
        self.file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, default=ROOT/'data/push_t/measured')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--capture', action='store_true',
                        help='Capture a fresh remote observation, disconnect, then simulate')
    parser.add_argument('--capture-timeout', type=float, default=90)
    add_remote_arguments(parser)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--limit', type=int, default=100)
    parser.add_argument('--benchmark', type=int, default=0)
    parser.add_argument('--seed', type=int, default=17)
    playback = parser.add_mutually_exclusive_group()
    playback.add_argument('--realtime', dest='realtime', action='store_true', default=True,
                          help='Play at physical simulation speed (default for the window)')
    playback.add_argument('--fast', dest='realtime', action='store_false',
                          help='Run the window as fast as possible instead of timed playback')
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    if args.capture:
        from lerobot_arm_control.tasks.push_t.push_t_capture import capture_snapshot, session_name

        args.output = args.output or ROOT/'data/push_t'/('capture-'+session_name())
        args.snapshot = capture_snapshot(args.output/'observations'/session_name(),
                                         host=args.host, remote_root=args.remote_root,
                                         timeout=args.capture_timeout)
    args.output = args.output or ROOT/'data/push_t/run'
    sim = PushSimulation(args.snapshot, args.output)
    if args.benchmark:
        result = benchmark(sim, args.benchmark, args.seed)
        (args.output/'benchmark.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({k: v for k, v in result.items() if k != 'cases'}))
    else:
        recorder = RolloutRecorder(args.output/'trajectory.jsonl')
        sim.record_callback = recorder.record
        recorder.record(sim)
        try:
            if args.headless:
                result = sim.solve(args.limit)
                (args.output/'result.json').write_text(json.dumps(result, indent=2))
                print(json.dumps({k: v for k, v in result.items() if k != 'trace'}))
            else:
                run_viewer(sim, args)
        finally:
            sim.record_callback = None
            recorder.close()


if __name__ == '__main__':
    main()
