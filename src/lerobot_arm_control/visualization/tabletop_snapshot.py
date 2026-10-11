"""Freeze the displayed table frame, both robot poses and their source parameters."""

import json

from lerobot_arm_control.core.files import atomic_json
from lerobot_arm_control.perception.tabletop_geometry import workspace_bounds
from lerobot_arm_control.visualization.tabletop_scene import scene_arms, scene_xml


def save_scene_snapshot(output, state, hello):
    """Keep the XML keyframe and saved parameters tied to the same observation."""
    config = hello['config']
    arms = {arm['arm']: {'registration': arm['registration'], 'qpos': arm['qpos'],
                         'joint_pose_source': arm['joint_pose_source'],
                         'feedback_status': (state.get('robot', {}) if arm['arm'] ==
                             config.get('robot', {}).get('arm') else
                             state.get('robot_feedback', {}).get(arm['arm'], {})).get(
                                 'status', 'disabled')}
            for arm in scene_arms(state, config)}
    parameters = {
        'version': 1, 'sequence': state['sequence'],
        'captured_unix_ns': state['captured_unix_ns'], 'units': state['units'],
        'reference': 'anchor AprilTag center; +X right; +Y up; +Z out',
        'table': {'anchor': {key: config[key] for key in (
            'dictionary', 'marker_id', 'marker_size_m')},
                  'workspace_bounds_m': state.get('appearance', {}).get(
                      'bounds', workspace_bounds(state, config)),
                  'extent_source': 'workspace patch; desk perimeter not measured',
                  'secondary_layouts': state.get('secondary_layouts', {})},
        'camera': {'device': hello.get('device'),
                   'position': state.get('camera_position'),
                   'rotation': state.get('camera_rotation'),
                   'controls': hello.get('camera_controls'),
                   'anchor_visible': state.get('anchor_visible'),
                   'pose_source': state.get('pose_source'),
                   'reprojection_error_px': state.get('reprojection_error_px')},
        'arms': arms, 'base_distances': state.get('base_distances', []),
        'objects': state['objects'], 'models': state['models'],
        'wood_height_measured': state['wood_height_measured'],
        'appearance': state.get('appearance'), 'config': config,
        'control_config_sha256': hello.get('control_config_sha256'),
        'joint_calibrations': {**hello.get('robot_feedback', {}),
                               **({config['robot']['arm']: hello['robot']}
                                  if hello.get('robot') else {})},
    }
    atomic_json(output/'state.json', state)
    atomic_json(output/'scene_parameters.json', parameters)
    model_path = output/'model.json'
    manifest = json.loads(model_path.read_text()) if model_path.exists() else {}
    manifest.update({key: parameters[key] for key in (
        'sequence', 'captured_unix_ns', 'units', 'models', 'arms', 'wood_height_measured')})
    if hello.get('robot'):
        manifest['robot'] = {**hello['robot'],
                             'registration': state.get('robot', {}).get('registration', {})}
    atomic_json(model_path, manifest)
    (output/'scene.xml').write_text(scene_xml(state, config, 'robot_assets'))
    return parameters
