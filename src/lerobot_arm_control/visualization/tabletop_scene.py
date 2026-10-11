"""Build a local MJCF from remotely estimated dimensions; no image processing."""

import math
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

from lerobot_arm_control.core.paths import resource_path, workspace_root
from lerobot_arm_control.perception.tabletop_geometry import workspace_bounds
from lerobot_arm_control.robot.so101.model_metadata import display_model

ROOT = workspace_root()


def scene_arms(state, config):
    """Keep follower names stable; additional arms require a localized base."""
    entries = [config.get('robot', {}), *config.get('additional_robots', [])]
    arms = []
    for index, entry in enumerate(entries):
        if not entry.get('enabled'):
            continue
        name = entry['arm']
        registration = state.get('base_registrations', {}).get(name, {})
        if index == 0:
            registration = state.get('robot', {}).get('registration', registration)
        elif 'position' not in registration:
            continue
        prefix = '' if index == 0 else name+'__'
        preview = entry.get('simulation', {}).get('preview_qpos', [0.]*6)
        measured = (state.get('robot', {}) if index == 0 else
                    state.get('robot_feedback', {}).get(name, {}))
        qpos = measured.get('qpos', preview) if measured.get('status') == 'live' else preview
        if len(qpos) != 6 or not all(math.isfinite(value) for value in qpos):
            raise ValueError(f'{name}: invalid six-joint simulation pose')
        arms.append({'arm': name, 'config': entry, 'prefix': prefix,
                     'root': 'robot_root' if index == 0 else prefix+'robot_root',
                     'registration': registration, 'qpos': list(qpos),
                     'joint_pose_source': (
                         'live_encoders_shared_calibration'
                         if measured.get('calibration_mode') == 'temporary_shared' else
                         'live_encoders') if measured.get('status') == 'live'
                     else 'unmeasured_preview',
                     'calibration_mode': measured.get('calibration_mode', 'uncalibrated'),
                     'calibration_source_arm': measured.get('calibration_source_arm'),
                     'independently_calibrated': measured.get('independently_calibrated', False)})
    return arms


def add_base_markers(body, arm):
    """Render exact decoded cells at the measured black-border corner positions."""
    import cv2
    import numpy as np

    for marker in arm['config'].get('markers', []):
        corners = marker.get('corners_base_m')
        if corners is None:
            continue
        corners = np.asarray(corners, float)
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, marker['dictionary']))
        cells = dictionary.markerSize+2
        pixels = cv2.aruco.generateImageMarker(dictionary, marker['id'], cells)
        right, down = corners[1]-corners[0], corners[3]-corners[0]
        side = np.linalg.norm(right)
        normal = np.cross(right, -down)/side**2
        yaw = math.atan2(right[1], right[0])
        for row in range(cells):
            for column in range(cells):
                center = (corners[0]+right*(column+.5)/cells
                          +down*(row+.5)/cells+normal*.0001)
                color = '1 1 1 1' if pixels[row, column] else '.015 .015 .015 1'
                ET.SubElement(body, 'geom',
                              name=f"{arm['prefix']}base_marker_{marker['id']}_{row}_{column}",
                              type='box', size=f'{side/cells/2} {side/cells/2} .0001',
                              pos=' '.join(map(str, center)), euler=f'0 0 {yaw}',
                              rgba=color, group='2', contype='0', conaffinity='0')


def add_tool_marker(chain, arm):
    """Display the nominal configured mount on the fixed finger, without actuators."""
    import cv2
    import numpy as np

    marker = arm['config'].get('tool_marker', {})
    reference = marker.get('cad_reference', {})
    mount = reference.get('marker_pose_in_gripper')
    if not marker.get('enabled') or not mount:
        return
    prefix = arm['prefix']
    finger = chain.find(f".//body[@name='{prefix}{reference['body']}']")
    rotation = np.asarray(mount['rotation'])
    name = f"{prefix}tool_marker_{marker['id']}"
    body = ET.SubElement(finger, 'body', name=name,
                         pos=' '.join(map(str, mount['position_m'])),
                         xyaxes=' '.join(map(str, rotation[:, :2].T.flatten())))
    ET.SubElement(body, 'site', name=name+'_center', pos='0 0 0', size='.001', group='5')
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, marker['dictionary']))
    cells = dictionary.markerSize+2
    pixels = cv2.aruco.generateImageMarker(dictionary, marker['id'], cells)
    side = marker['marker_size_m']
    for row in range(cells):
        for column in range(cells):
            x = side*((column+.5)/cells-.5)
            y = side*(.5-(row+.5)/cells)
            ET.SubElement(body, 'geom', name=f'{name}_{row}_{column}', type='box',
                          pos=f'{x} {y} .0001', size=f'{side/cells/2} {side/cells/2} .0001',
                          rgba='1 1 1 1' if pixels[row, column] else '.015 .015 .015 1',
                          mass='0', group='2', contype='0', conaffinity='0')


def scene_xml(state, config, robot_mesh_dir=None):
    scale = config["marker_size_m"] or 1.0
    root = ET.Element("mujoco", model="Live tabletop - AprilTag coordinates")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", gravity="0 0 0")
    visual = ET.SubElement(root, "visual")
    appearance = state.get("appearance", {})
    lighting = appearance.get("lighting", {})
    ET.SubElement(visual, "headlight", ambient=" ".join(map(str, lighting.get(
        "headlight_ambient", [.5, .5, .5]))), diffuse="0 0 0", specular="0 0 0")
    ET.SubElement(visual, "quality", shadowsize="4096", offsamples="4")
    ET.SubElement(visual, "rgba", haze="0.16 0.18 0.20 1")
    assets = ET.SubElement(root, "asset")
    background = appearance.get("background_rgb", [.3, .32, .34])
    ET.SubElement(assets, "texture", name="room_background", type="skybox", builtin="gradient",
                  width="64", height="384", rgb1=" ".join(map(str, background)),
                  rgb2=" ".join(map(str, [value*.6 for value in background])))
    ET.SubElement(assets, "texture", name="tag", type="2d", file="marker.png")
    ET.SubElement(assets, "material", name="tag", texture="tag", texrepeat="1 1")
    for filename in appearance.get("texture_files", []):
        name = Path(filename).stem
        ET.SubElement(assets, "texture", name="observed_"+name, type="2d",
                      file="appearance/"+filename, vflip="false")
        ET.SubElement(assets, "material", name="observed_"+name,
                      texture="observed_"+name, texrepeat="1 1", texuniform="false",
                      rgba="1 1 1 1", specular="0.04", shininess="0.12", reflectance="0")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", name="observed_key", directional="true",
                  pos=" ".join(map(str, lighting.get("main_pos", [-.4, .6, 1.4]))),
                  dir=" ".join(map(str, lighting.get("main_dir", [.5, -.5, -1.4]))),
                  diffuse=" ".join(map(str, lighting.get("main_diffuse", [.32, .31, .3]))),
                  ambient="0 0 0", specular="0.12 0.12 0.12", castshadow="true")
    ET.SubElement(world, "light", name="observed_fill", directional="true",
                  pos="0 -1 1.8", dir="0 .6 -1.8", ambient="0 0 0",
                  diffuse=" ".join(map(str, lighting.get("fill_diffuse", [.1, .11, .13]))),
                  specular="0.04 0.04 0.04", castshadow="false")
    # A plane patch covering the reconstructed workspace, not a measured desk perimeter.
    arms = scene_arms(state, config)
    if arms:
        robot = ET.parse(resource_path('assets/so101/so101.xml')).getroot()
        root.append(deepcopy(robot.find("default")))
        mesh_dir = Path(robot_mesh_dir or resource_path('assets/so101/assets'))
        mesh_names = set()
        for arm in arms:
            robot = ET.parse(display_model(arm['config'].get(
                'simulation', {}).get('end_effector', 'gripper'))).getroot()
            for source in robot.find('asset'):
                if source.tag != 'mesh':
                    continue
                name = source.get('name', Path(source.get('file')).stem)
                if name in mesh_names:
                    continue
                mesh_names.add(name)
                asset = deepcopy(source)
                asset.set('name', name)
                asset.set('file', str(mesh_dir / asset.get('file')))
                assets.append(asset)
            prefix = arm['prefix']
            materials = {}
            color = arm['config'].get('simulation', {}).get('plastic_rgb',
                appearance.get('robot_plastic_rgb', [.055, .06, .065]))
            for source in robot.find('asset'):
                if source.tag != 'material':
                    continue
                asset = deepcopy(source)
                name = source.get('name')
                materials[name] = prefix+name
                asset.set('name', materials[name])
                motor = 'sts3215' in name
                observed_primary = not prefix and config.get('appearance', {}).get('enabled')
                if observed_primary or (prefix and not motor):
                    asset.set('rgba', ' '.join(map(str, [*color, 1])))
                asset.set('specular', '0.22' if motor else '0.07')
                asset.set('shininess', '0.25' if motor else '0.08')
                assets.append(asset)
            registration = arm['registration']
            body = ET.SubElement(world, 'body', name=arm['root'], mocap='true',
                                 pos=' '.join(map(str, registration.get(
                                     'position', [0, .32, .0024]))),
                                 quat=' '.join(map(str, registration.get(
                                     'quaternion', [1, 0, 0, 0]))))
            chain = deepcopy(robot.find('worldbody/body'))
            for element in chain.iter():
                if prefix and element.get('name'):
                    element.set('name', prefix+element.get('name'))
                if element.get('material'):
                    element.set('material', materials[element.get('material')])
            add_tool_marker(chain, arm)
            body.append(chain)
            add_base_markers(body, arm)
        # This scene only runs mj_forward; no actuator or physics command is sent to hardware.
    xmin, xmax, ymin, ymax = appearance.get("bounds", workspace_bounds(state, config))
    ET.SubElement(world, "geom", name="tabletop_workspace", type="box",
                  pos=f"{(xmin+xmax)/2} {(ymin+ymax)/2} {-scale*.04}",
                  size=f"{(xmax-xmin)/2} {(ymax-ymin)/2} {scale*.04}",
                  rgba="1 1 1 1" if appearance else "0.88 0.9 0.91 1",
                  **({"material": "observed_tabletop"} if appearance else {}),
                  contype="0", conaffinity="0")
    ET.SubElement(world, "geom", name="anchor_tag", type="plane", material="tag",
                  size=f"{scale/2} {scale/2} .001", pos=f"0 0 {scale*.001}",
                  contype="0", conaffinity="0")
    # Auxiliary marker is a stationary desk print, so keep its measured position on the patch.
    for index, corners in enumerate(state.get("secondary_layouts", {}).values()):
        if appearance and "secondary_tag.png" in appearance["texture_files"]:
            center = [sum(p[axis] for p in corners)/4 for axis in (0, 1)]
            dx, dy = corners[1][0]-corners[0][0], corners[1][1]-corners[0][1]
            yaw = math.atan2(dy, dx)
            ET.SubElement(world, "geom", name=f"secondary_tag_{index}", type="plane",
                          material="observed_secondary_tag", size=f"{scale/2} {scale/2} .001",
                          pos=f"{center[0]} {center[1]} {scale*.001}",
                          quat=f"{math.cos(yaw/2)} 0 0 {math.sin(yaw/2)}",
                          contype="0", conaffinity="0")
    for name, dimensions in state["models"].items():
        obj = state["objects"].get(name, {"xy": [0, 0], "yaw": 0})
        # Unmeasured wood is a thin display silhouette, not an inferred 3D thickness.
        height = dimensions["height"] or scale * .002
        angle = obj["yaw"]
        body = ET.SubElement(world, "body", name=name, mocap="true",
                             pos=f'{obj["xy"][0]} {obj["xy"][1]} {height/2+scale*.002}',
                             quat=f"{math.cos(angle/2)} 0 0 {math.sin(angle/2)}")
        width, length = dimensions["width"], dimensions["length"]
        cap, stem = dimensions["cap_depth"], dimensions["stem_width"]
        alpha = 1 if name in state["objects"] else 0
        color = (f"0.06 0.06 0.07 {alpha}" if name == "black_t"
                 else f"0.76 0.63 0.28 {alpha}")
        if appearance:
            color = f"1 1 1 {alpha}"
        ET.SubElement(body, "geom", name=name+"_cap", type="box",
                      pos=f"0 {(length-cap)/2} 0",
                      size=f"{width/2} {cap/2} {height/2}", rgba=color,
                      **({"material": "observed_"+name+"_cap"} if appearance else {}),
                      contype="0", conaffinity="0")
        ET.SubElement(body, "geom", name=name+"_stem", type="box",
                      pos=f"0 {-cap/2} 0",
                      size=f"{stem/2} {(length-cap)/2} {height/2}", rgba=color,
                      **({"material": "observed_"+name+"_stem"} if appearance else {}),
                      contype="0", conaffinity="0")
    if arms:
        keyframes = ET.SubElement(root, "keyframe")
        ET.SubElement(keyframes, "key", name="recorded_pose",
                      qpos=" ".join(str(value) for arm in arms for value in arm['qpos']))
    ET.indent(root)
    return ET.tostring(root, encoding="unicode") + "\n"


def update_scene_robots(model, data, state, config, age):
    """Update calibrated bases independently; unmeasured joints stay translucent."""
    primary = config.get('robot', {}).get('arm')
    table_live = state.get('marker_visible') and (
        age+state.get('capture_age_at_send_ms', 0)/1000 < .5)
    for entry in [config.get('robot', {}), *config.get('additional_robots', [])]:
        if not entry.get('enabled'):
            continue
        name = entry['arm']
        root_name = 'robot_root' if name == primary else name+'__robot_root'
        try:
            body = model.body(root_name).id
        except KeyError:
            continue
        robot = (state.get('robot', {}) if name == primary else
                 state.get('robot_feedback', {}).get(name, {}))
        live = (robot.get('status') == 'live' and age < .5
                and age+robot.get('sample_age_at_send_ms', 1000)/1000 < .5)
        registration = state.get('base_registrations', {}).get(name, {})
        if name == primary:
            registration = robot.get('registration', registration)
        placed = registration.get('status') == 'fixed' or (
            table_live and registration.get('status') in ('tracked', 'held'))
        estimated = table_live and registration.get('status') in ('visual_estimate',
                                                                 'visual_cached')
        if placed or estimated:
            mocap = model.body_mocapid[body]
            data.mocap_pos[mocap] = registration['position']
            data.mocap_quat[mocap] = registration['quaternion']
        if live:
            for joint, angle in zip(robot['joints'], robot['qpos'], strict=True):
                prefix = '' if name == primary else name+'__'
                data.qpos[model.joint(prefix+joint).qposadr[0]] = angle
        alpha = 1. if live and (placed or estimated) else .4 if live else .25 if placed else .15
        if live and robot.get('calibration_mode') == 'temporary_shared':
            alpha = min(alpha, .8)
        visuals = (model.geom_group == 2) & (model.body_rootid[model.geom_bodyid] == body)
        model.geom_rgba[visuals, 3] = alpha
