"""Local fixed-jaw CAD inspection and the user-specified nominal ID 8 mount."""

import hashlib
import json

import numpy as np

from lerobot_arm_control.robot.so101.model_metadata import MODEL


def cad_reference(model_path=MODEL):
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body = model.body('gripper').id
    mesh_name = 'wrist_roll_follower_so101_v1'
    mesh = model.mesh(mesh_name).id
    geom = next(i for i in range(model.ngeom)
                if model.geom_bodyid[i] == body
                and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
                and model.geom_dataid[i] == mesh)
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    vertices = model.mesh_vert[start:start+count].astype(float)
    # MuJoCo centers and rotates imported meshes. Undo through the compiled geom,
    # then express vertices in the gripper body rather than treating them as STL coordinates.
    world = vertices@data.geom_xmat[geom].reshape(3, 3).T+data.geom_xpos[geom]
    vertices = (world-data.xpos[body])@data.xmat[body].reshape(3, 3)
    start, count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
    triangles = vertices[model.mesh_face[start:start+count]]
    plane_x = vertices[:, 0].min()
    tolerance = 1e-6
    surface = triangles[np.all(np.abs(triangles[:, :, 0]-plane_x) < tolerance, axis=1)]
    # The outer holder has a 3 mm shoulder fillet. Include it when finding the
    # physical upper edge, rather than measuring from the inset flat patch.
    upper_edge = vertices[vertices[:, 0] <= plane_x+.00301, 2].max()
    symmetry_y = (surface[:, :, 1].min()+surface[:, :, 1].max())/2
    center = np.round([plane_x, symmetry_y, upper_edge-.011], 6)
    # Decoded code orientation in the installation photograph: marker +X toward
    # the wrist (+Z body), +Y along +Y body, normal outward (-X body).
    rotation = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    contact = model.geom('fixed_jaw_box3')
    frame = model.site('gripperframe')
    assert contact.bodyid[0] == body and frame.bodyid[0] == body
    return {
        'model_file': 'assets/so101/so101.xml',
        'model_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
        'fixed_jaw_mesh': mesh_name,
        'mesh_sha256': hashlib.sha256(
            (model_path.parent/'assets'/f'{mesh_name}.stl').read_bytes()).hexdigest(),
        'body': 'gripper',
        'coordinates': 'meters in gripper body frame; not table coordinates',
        'outer_flat_surface': {
            'normal': [-1, 0, 0], 'plane_x_m': round(float(plane_x), 6),
            'mesh_tolerance_m': tolerance,
            'patch_bounds_m': np.round(
                [surface.min(axis=(0, 1)), surface.max(axis=(0, 1))], 6).tolist(),
            'upper_rounded_edge_z_m': round(float(upper_edge), 6),
            'symmetry_center_y_m': round(float(symmetry_y), 6),
            'installation_correspondence': 'User-confirmed View from X- face',
        },
        'fixed_jaw_contact_reference': {
            'geom': 'fixed_jaw_box3', 'position_m': contact.pos.tolist(),
            'definition': 'Contact capsule center, not its physical contact surface',
        },
        'gripperframe': {'position_m': frame.pos.tolist(),
                         'quaternion_wxyz': frame.quat.tolist()},
        'visual_tip_min_z_m': round(float(vertices[:, 2].min()), 6),
        'marker_pose_in_gripper': {
            'position_m': center.tolist(), 'rotation': rotation.tolist(),
            'center_distance_from_upper_edge_m': .011,
            'upper_edge_definition': 'Rounded holder shoulder on the +Z (wrist) side',
            'position_source': ('User-specified symmetry center and 11 mm edge distance; '
                                'CAD surface'),
            'rotation_source': 'Code axes inferred from installation photograph; not validated',
            'sticker_thickness_m': 0,
            'thickness_measured': False,
            'transform_validated': False,
        },
        'contact_offset_marker_m': np.round((contact.pos-center)@rotation, 6).tolist(),
    }


def main():
    import argparse

    argparse.ArgumentParser(description=__doc__).parse_args()
    print(json.dumps(cad_reference(), indent=2))


if __name__ == '__main__':
    main()
