"""Fresh capture selection and freezing, with no network or physical hardware."""

import copy
import json
import shutil
from types import SimpleNamespace

import pytest

from lerobot_arm_control.planning.push_t import PushPlanner, RolloutRecorder, pose
from lerobot_arm_control.tasks.push_t import push_t_capture as capture


@pytest.fixture
def observed(tmp_path):
    source = capture.ROOT/'data/push_t/measured'
    shutil.copytree(source/'appearance', tmp_path/'appearance')
    state = json.loads((source/'state.json').read_text())
    state['capture_age_at_send_ms'] = 100
    state['robot']['sample_age_at_send_ms'] = 20
    state['robot']['registration']['pose_age_s'] = 0
    return state, tmp_path


@pytest.mark.parametrize('problem', ['camera', 'motor', 'cached_target', 'held_wood',
                                     'missing_base', 'uncalibrated_scale', 'nan_joint'])
def test_rejects_old_cached_or_incomplete_initialization(observed, problem):
    state, output = observed
    assert capture.observation_problem(state, .02, output) is None
    if problem == 'camera':
        state['capture_age_at_send_ms'] = 510
    elif problem == 'motor':
        state['robot']['sample_age_at_send_ms'] = 510
    elif problem == 'cached_target':
        state['objects']['black_t'].update(observed=False, tracking_status='static_cached')
    elif problem == 'held_wood':
        state['objects']['wood_t'].update(observed=False, tracking_status='held')
    elif problem == 'missing_base':
        state['robot']['registration'].pop('position')
    elif problem == 'uncalibrated_scale':
        state['units'] = 'marker'
    else:
        state['robot']['qpos'][0] = float('nan')
    assert capture.observation_problem(state, .02, output)


def test_fresh_partial_contour_can_initialize_known_complete_t(observed):
    state, output = observed
    state['objects']['wood_t']['tracking_status'] = 'partial'
    assert capture.observation_problem(state, .02, output) is None
    assert capture.observation_problem(state, .6, output)


def test_saved_fixed_base_initializes_without_visual_age_but_requires_fresh_feedback(observed):
    state, output = observed
    base = state['robot']['registration']
    base.update(status='fixed', fixed_to_table=True, calibrated=True,
                pose_source='fixed_marker_calibration', calibration_age_s=86400)
    base.pop('pose_age_s')
    assert capture.observation_problem(state, .02, output) is None
    base['fixed_to_table'] = False
    assert capture.observation_problem(state, .02, output)
    base['fixed_to_table'] = True
    state['robot']['sample_age_at_send_ms'] = 600
    assert capture.observation_problem(state, .02, output)


def test_calibration_must_match_identity_and_remote_angle_fit():
    source = capture.ROOT/'data/push_t/measured'
    hello = json.loads((source/'calibration.json').read_text())
    path = source/'joint_calibration.json'
    assert capture.calibration_bytes(hello, path) == path.read_bytes()
    hello['robot']['fits']['gripper']['zero_raw'] += 1
    with pytest.raises(ValueError, match='calibrations differ'):
        capture.calibration_bytes(hello, path)


def test_capture_freezes_selected_frame_and_closes_before_return(observed, monkeypatch):
    state, output = observed
    source = capture.ROOT/'data/push_t/measured'
    hello = json.loads((source/'calibration.json').read_text())
    state.update(frame='selected.jpg', annotated='selected-annotated.jpg',
                 native_frame='selected.jpg')
    (output/'selected.jpg').write_bytes(b'fresh selected frame')
    (output/'selected-annotated.jpg').write_bytes(b'fresh selected annotation')
    closed = []

    class FakeLink:
        def __init__(self, host, remote_root, directory):
            self.process = SimpleNamespace(poll=lambda: None)

        def snapshot(self):
            return hello, (state, b''), capture.time.monotonic(), 1, None

        def close(self):
            closed.append(True)
            state['objects']['wood_t']['xy'][0] += .1

    initial = copy.deepcopy(state['objects']['wood_t']['xy'])
    monkeypatch.setattr(capture, 'TableLink', FakeLink)
    result = capture.capture_snapshot(output, timeout=1,
                                     calibration=source/'joint_calibration.json')
    assert closed == [True]
    frozen = json.loads((result/'state.json').read_text())
    assert frozen['objects']['wood_t']['xy'] == initial
    assert frozen['initialization']['remote_closed_before_simulation']
    assert (result/'frame.jpg').read_bytes() == b'fresh selected frame'


def test_recording_includes_execution_and_excludes_planning(tmp_path):
    from lerobot_arm_control.planning.push_t import PushSimulation

    sim = PushSimulation(output=tmp_path/'sim')
    path = tmp_path/'trajectory.jsonl'
    recorder = RolloutRecorder(path)
    sim.record_callback = recorder.record
    recorder.record(sim)
    planner = PushPlanner(sim)
    planner.capture(sim)
    action = planner.plan().action
    assert action is not None
    assert len(path.read_text().splitlines()) == 1
    sim.execute(action)
    recorder.close()
    frames = [json.loads(line) for line in path.read_text().splitlines()]
    assert len(frames) > 10
    assert all(len(frame['qpos']) == 13 for frame in frames)
    assert all(b['time_s'] > a['time_s'] for a, b in zip(frames, frames[1:]))
    assert max(b['time_s']-a['time_s'] for a, b in zip(frames, frames[1:])) < .019
    assert frames[-1]['wood_pose'] == pytest.approx(pose(sim.data), abs=.002)


def test_invalid_capture_closes_connection_without_falling_back(observed, monkeypatch):
    state, output = observed
    state['objects']['wood_t'].update(observed=False, tracking_status='held')
    closed = []

    class FakeLink:
        def __init__(self, host, remote_root, directory):
            self.process = SimpleNamespace(poll=lambda: None)

        def snapshot(self):
            return {}, (state, b''), capture.time.monotonic(), 1, None

        def close(self):
            closed.append(True)

    monkeypatch.setattr(capture, 'TableLink', FakeLink)
    with pytest.raises(TimeoutError, match='visible wood_t'):
        capture.capture_snapshot(output, timeout=.01)
    assert closed == [True]
    assert not (output/'state.json').exists()
