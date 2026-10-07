"""Local extraction of observed plane textures and approximate scene lighting.

Only visible pixels are measured. Inpainted/hidden regions and lighting are
explicit estimates. CAD masking and texture extraction run on the simulation host.
"""

import base64
import math

import cv2
import mujoco
import numpy as np

from .tabletop_geometry import workspace_bounds


def encode_png(picture):
    ok, data = cv2.imencode(".png", picture)
    if not ok:
        raise RuntimeError("Cannot encode observed texture")
    return base64.b64encode(data).decode("ascii")


class ObservedAppearance:
    def __init__(self, config):
        self.config = config
        self.metadata = None
        self.matrix = np.asarray(config["camera_matrix"])
        self.distortion = np.asarray(config["dist_coeffs"])
        self.robot_model = self.robot_data = None
        if config.get("robot", {}).get("enabled"):
            from pathlib import Path

            path = Path(__file__).resolve().parents[2] / "assets/so101/so101.xml"
            self.robot_model = mujoco.MjModel.from_xml_path(str(path))
            self.robot_data = mujoco.MjData(self.robot_model)

    def project(self, world, state, native):
        rotation = np.asarray(state["camera_rotation"])
        center = np.asarray(state["camera_position"])
        pixels = cv2.projectPoints(
            np.asarray(world, dtype=float),
            cv2.Rodrigues(rotation)[0],
            -rotation @ center,
            self.matrix,
            self.distortion,
        )[0]
        pixels = pixels.reshape(-1, 2)
        if native:
            affine = np.asarray(self.config["low_to_capture_affine"])
            pixels = pixels @ affine[:, :2].T + affine[:, 2]
        return pixels

    def sample(self, frame, world, state, shape, native):
        pixels = self.project(world, state, native).reshape(*shape, 2)
        valid = (
            (pixels[:, :, 0] >= 1)
            & (pixels[:, :, 0] < frame.shape[1] - 1)
            & (pixels[:, :, 1] >= 1)
            & (pixels[:, :, 1] < frame.shape[0] - 1)
        )
        picture = cv2.remap(
            frame,
            pixels[:, :, 0].astype(np.float32),
            pixels[:, :, 1].astype(np.float32),
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        return picture, valid

    def robot_mask(self, frame, state, native):
        mask = np.zeros(frame.shape[:2], np.uint8)
        robot = state.get("robot", {})
        pose = robot.get("registration", {})
        if self.robot_model is None or "position" not in pose or "qpos" not in robot:
            return mask
        self.robot_data.qpos[:] = robot["qpos"]
        mujoco.mj_forward(self.robot_model, self.robot_data)
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, np.asarray(pose["quaternion"]))
        rotation = rotation.reshape(3, 3)
        for geom in np.flatnonzero(self.robot_model.geom_group == 2):
            mesh = self.robot_model.geom_dataid[geom]
            address, count = (
                self.robot_model.mesh_vertadr[mesh],
                self.robot_model.mesh_vertnum[mesh],
            )
            points = self.robot_model.mesh_vert[address : address + count]
            points = (
                points @ self.robot_data.geom_xmat[geom].reshape(3, 3).T
                + self.robot_data.geom_xpos[geom]
            )
            world = points @ rotation.T + pose["position"]
            pixels = self.project(world, state, native).astype(np.float32)
            cv2.fillConvexPoly(mask, np.int32(cv2.convexHull(pixels)), 255)
        radius = 25 if native else 11
        return cv2.dilate(mask, np.ones((radius, radius), np.uint8))

    def extract(self, frame, state, native=False):
        if self.metadata or not state["marker_visible"]:
            return None
        if any(name not in state["objects"] for name in ("black_t", "wood_t")):
            return None
        if self.config.get("robot", {}).get("enabled") and "position" not in (
            state.get("robot", {}).get("registration", {})
        ):
            return None
        xmin, xmax, ymin, ymax = workspace_bounds(state, self.config)
        size = self.config.get("appearance", {}).get("texture_size", 1024)
        # PNG row zero is world +Y, matching MuJoCo's image-loading convention.
        x, y = np.meshgrid(np.linspace(xmin, xmax, size), np.linspace(ymax, ymin, size))
        world = np.column_stack((x.ravel(), y.ravel(), np.zeros(size * size)))
        tabletop, visible = self.sample(frame, world, state, (size, size), native)
        hsv = cv2.cvtColor(tabletop, cv2.COLOR_BGR2HSV)
        static = visible & (hsv[:, :, 2] > 145) & (hsv[:, :, 1] < 65)
        static &= ~((hsv[:, :, 0] < 25) & (hsv[:, :, 1] > 25))
        projected_robot, _ = self.sample(
            self.robot_mask(frame, state, native), world, state, (size, size), native
        )
        static &= projected_robot == 0

        def polygon_mask(points, padding=0):
            points = np.asarray(points)
            pixels = np.column_stack(
                (
                    (points[:, 0] - xmin) / (xmax - xmin) * (size - 1),
                    (ymax - points[:, 1]) / (ymax - ymin) * (size - 1),
                )
            )
            mask = np.zeros((size, size), np.uint8)
            cv2.fillPoly(mask, [np.int32(pixels)], 1)
            if padding:
                mask = cv2.dilate(mask, np.ones((padding, padding), np.uint8))
            return mask != 0

        half = self.config["marker_size_m"] / 2
        supported_points = [[-half, -half], [half, -half], [half, half], [-half, half]]
        for obj in state["objects"].values():
            supported_points.extend(obj["complete_outline_xy"])
            static[polygon_mask(obj["complete_outline_xy"], 19)] = False
        for corners in state.get("secondary_layouts", {}).values():
            supported_points.extend(np.asarray(corners)[:, :2].tolist())
            static[polygon_mask(np.asarray(corners)[:, :2], 7)] = False
        half = self.config["marker_size_m"] / 2
        static[polygon_mask([[-half, -half], [half, -half], [half, half], [-half, half]], 7)] = (
            False
        )
        base = state.get("robot", {}).get("registration", {}).get("position")
        if base:
            # Remove the fixed base and its low nearby shadow before drawing dynamic shadows.
            static[((x - base[0]) ** 2 + (y - base[1]) ** 2) < 0.12**2] = False
        if base:
            supported_points.extend(
                [[base[0] + dx, base[1] + dy] for dx in (-0.08, 0.08) for dy in (-0.08, 0.08)]
            )
        hull = cv2.convexHull(np.asarray(supported_points, dtype=np.float32)).reshape(-1, 2)
        static &= polygon_mask(hull)
        # Only table/paper pixels survive: hands, dark CAD, tags and T silhouettes are excluded.
        observed = int(static.sum())
        if observed < size * size * 0.15:
            return None
        median = np.median(tabletop[static], axis=0)
        # Large hidden regions use normalized smooth interpolation from *only* valid pixels.
        # Telea on large holes can drag hands/clutter into colored ghost silhouettes.
        weights = static.astype(np.float32)
        smoothed_weights = cv2.GaussianBlur(weights, (0, 0), 24)
        weighted_color = cv2.GaussianBlur(
            tabletop.astype(np.float32) * weights[:, :, None], (0, 0), 24
        )
        fill = weighted_color / np.maximum(smoothed_weights[:, :, None], 0.0001)
        confidence = np.clip(smoothed_weights / 0.04, 0, 1)[:, :, None]
        fill = fill * confidence + median * (1 - confidence)
        clean = tabletop.astype(np.float32)
        clean[~static] = fill[~static]
        coverage = cv2.GaussianBlur(polygon_mask(hull).astype(np.float32), (0, 0), 30)
        clean = clean * coverage[:, :, None] + median * (1 - coverage[:, :, None])
        clean = np.uint8(np.clip(cv2.GaussianBlur(clean, (0, 0), 0.8), 0, 255))
        # Preserve paper/material edges while suppressing isolated baked shadow/exposure gradients.
        gray = cv2.cvtColor(clean, cv2.COLOR_BGR2GRAY).astype(float)
        samples = static & (hsv[:, :, 2] > 180)
        if samples.sum() < 200:
            samples = static
        coordinates = np.column_stack(
            (
                np.ones(samples.sum()),
                (x[samples] - (xmin + xmax) / 2) / (xmax - xmin),
                (y[samples] - (ymin + ymax) / 2) / (ymax - ymin),
            )
        )
        coefficients = np.linalg.lstsq(coordinates, gray[samples], rcond=None)[0]
        gradient = coefficients[1:]
        magnitude = np.linalg.norm(gradient)
        direction = gradient / magnitude if magnitude > 2 else np.array([0.7, -0.7])
        light_position = [
            float((xmin + xmax) / 2 + 0.8 * direction[0]),
            float((ymin + ymax) / 2 + 0.8 * direction[1]),
            1.4,
        ]
        light_direction = [-float(direction[0]) * 0.8, -float(direction[1]) * 0.8, -1.4]
        # Classic MuJoCo Phong illumination; normalize albedo once to avoid double exposure.
        plane_gain = 0.50 + 0.31 * (1.4 / math.sqrt(1.4**2 + 0.8**2)) + 0.11 * 0.9
        # Separate broad material regions from photograph lighting. Retain only fine detail
        # so neither an exposure gradient nor a silhouette-shaped fill becomes a baked stain.
        bright = static & (hsv[:, :, 2] >= np.percentile(hsv[:, :, 2][static], 65))
        desk_color = np.median(tabletop[bright], axis=0)
        albedo = np.empty_like(clean, dtype=np.float32)
        albedo[:] = desk_color
        paper = (
            np.uint8(static & (hsv[:, :, 0] > 65) & (hsv[:, :, 0] < 140) & (hsv[:, :, 1] > 15))
            * 255
        )
        paper = cv2.morphologyEx(paper, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        contours = cv2.findContours(paper, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        paper_detected = False
        if contours:
            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) > size * size * 0.015:
                paper_color = np.median(tabletop[(paper != 0) & static], axis=0)
                patch = np.zeros((size, size), np.uint8)
                cv2.fillConvexPoly(patch, np.int32(cv2.boxPoints(cv2.minAreaRect(contour))), 255)
                albedo[patch != 0] = paper_color
                paper_detected = True
        fine = np.clip(
            clean.astype(np.float32) - cv2.GaussianBlur(clean.astype(np.float32), (0, 0), 8), -4, 4
        )
        fine *= cv2.GaussianBlur(static.astype(np.float32), (0, 0), 3)[:, :, None]
        clean = np.uint8(np.clip((albedo + fine) / plane_gain, 0, 255))
        textures = {"tabletop.png": encode_png(clean)}
        for key in state.get("secondary_layouts", {}):
            dictionary, marker_id = key.rsplit(":", 1)
            textures["secondary_tag.png"] = encode_png(
                cv2.aruco.generateImageMarker(
                    cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)),
                    int(marker_id),
                    256,
                )
            )
        materials = {}
        for name, obj in state["objects"].items():
            if name not in ("black_t", "wood_t"):
                continue
            model = state["models"][name]
            angle = obj["yaw"]
            rotation = np.array(
                [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
            )
            for part, width, length, cy in (
                (
                    "cap",
                    model["width"],
                    model["cap_depth"],
                    (model["length"] - model["cap_depth"]) / 2,
                ),
                (
                    "stem",
                    model["stem_width"],
                    model["length"] - model["cap_depth"],
                    -model["cap_depth"] / 2,
                ),
            ):
                columns, rows = max(64, round(width * 6000)), max(64, round(length * 6000))
                xx, yy = np.meshgrid(
                    np.linspace(-width / 2, width / 2, columns),
                    np.linspace(length / 2, -length / 2, rows) + cy,
                )
                xy = np.column_stack((xx.ravel(), yy.ravel())) @ rotation.T + obj["xy"]
                height = model["height"]
                pixels = np.column_stack((xy, np.full(len(xy), height)))
                picture, valid = self.sample(frame, pixels, state, (rows, columns), native)
                colors = cv2.cvtColor(picture, cv2.COLOR_BGR2HSV)
                if name == "wood_t":
                    valid &= (
                        (colors[:, :, 0] >= 15)
                        & (colors[:, :, 0] <= 42)
                        & (colors[:, :, 1] > 35)
                        & (colors[:, :, 2] > 90)
                    )
                else:
                    valid &= colors[:, :, 2] < 110
                if valid.sum() < 50:
                    return None
                color = np.median(picture[valid], axis=0)
                picture[~valid] = color
                picture = np.uint8(np.clip(picture.astype(float) / plane_gain, 0, 255))
                textures[name + "_" + part + ".png"] = encode_png(picture)
                materials[name + "_" + part] = (color[::-1] / 255 / plane_gain).clip(0, 1).tolist()
        # Color comes from observed dark pixels in the physical robot's image footprint.
        robot_color = [0.055, 0.06, 0.065]
        if base:
            location = self.project([base], state, native)[0]
            u, v = np.int32(location)
            radius = 100 if native else 40
            roi = frame[max(0, v - radius) : v + radius, max(0, u - radius) : u + radius]
            if roi.size:
                dark = roi[cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) < 65]
                if len(dark) > 30:
                    robot_color = (
                        (np.quantile(dark, 0.70, axis=0)[::-1] / 255 / plane_gain)
                        .clip(0.025, 0.15)
                        .tolist()
                    )
        self.metadata = {
            "version": 1,
            "source_sequence": state["sequence"],
            "source_captured_unix_ns": state["captured_unix_ns"],
            "perception_location": "remote",
            "bounds": [xmin, xmax, ymin, ymax],
            "texture_rows": "world +Y to -Y; MuJoCo PNG convention, vflip=false",
            "materials_rgb": materials,
            "robot_plastic_rgb": robot_color,
            "background_rgb": (
                np.median(frame[: frame.shape[0] // 4, : frame.shape[1] // 5], axis=(0, 1))[::-1]
                / 255
            ).tolist(),
            "observed_table_fraction": observed / (size * size),
            "paper_material_detected": paper_detected,
            "texture_method": "observed desk/paper colors and fine photo detail; "
                              "broad lighting removed",
            "hidden_surfaces": "inpainted or uniform estimated material",
            "lighting": {
                "headlight_ambient": [0.5, 0.5, 0.5],
                "main_pos": light_position,
                "main_dir": light_direction,
                "main_diffuse": [0.32, 0.31, 0.30],
                "fill_diffuse": [0.10, 0.11, 0.13],
                "plane_gain": plane_gain,
                "source": "robust bright-table exposure gradient; approximate",
                "physically_calibrated": False,
            },
            "texture_files": list(textures),
        }
        return {"metadata": self.metadata, "pngs": textures}
