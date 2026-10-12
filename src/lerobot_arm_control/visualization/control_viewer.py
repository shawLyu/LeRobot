"""MuJoCo renderer with exclusive keyboard handling, without Simulate shortcuts."""

from contextlib import nullcontext
from functools import lru_cache
from pathlib import Path

import glfw
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


@lru_cache(maxsize=512)
def panel_lines(line, font, width):
    """Cache layout; measure whole lines instead of every growing character prefix."""
    if font.getlength(line) <= width:
        return (line,)
    lines = []
    while line:
        low, high = 1, len(line)
        while low < high:
            middle = (low+high+1)//2
            if font.getlength(line[:middle]) <= width:
                low = middle
            else:
                high = middle-1
        end = line.rfind(' ', 0, low+1) if low < len(line) else -1
        if end <= 0:
            end = low
        lines.append(line[:end])
        line = line[end:].lstrip()
    return tuple(lines)


class ControlViewer:
    def __init__(self, model, data, key_callback, title, *, hold_on_focus_loss=True):
        if not glfw.init():
            raise RuntimeError("Cannot initialize local GLFW")
        self.window = glfw.create_window(1280, 800, title, None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Cannot create MuJoCo window")
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)
        self.model, self.data = model, data
        self.key_callback = key_callback
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        # Visual groups 0/1/2 only. Numeric keys never toggle geometry groups.
        self.opt.geomgroup[:] = (1, 1, 1, 0, 0, 0)
        self.scene = mujoco.MjvScene(model, maxgeom=2000)
        self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self.text = ""
        self.images = []
        self.last_cursor = None
        self.panel_width = 0
        self.reset_status = None
        self.buttons = []
        self.font_path = next((path for path in (
            Path("/System/Library/Fonts/Supplemental/Arial.ttf"),
            Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
            Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
            Path("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"),
        ) if path.exists()), None)
        if self.font_path is None:
            self.close()
            raise RuntimeError("No usable system font found")
        self.font_size = None
        self.panel_key = None
        self.panel_pixels = None
        glfw.set_key_callback(self.window, self.on_key)
        glfw.set_cursor_pos_callback(self.window, self.on_cursor)
        glfw.set_scroll_callback(self.window, self.on_scroll)
        glfw.set_mouse_button_callback(self.window, self.on_mouse_button)
        # Hold on focus loss; background keyboard events must not move the arm.
        if hold_on_focus_loss:
            glfw.set_window_focus_callback(
                self.window, lambda window, focused: self.key_callback(glfw.KEY_SPACE)
                if not focused else None)

    def on_key(self, window, key, scancode, action, mods):
        if mods or action not in (glfw.PRESS, glfw.REPEAT):
            return
        if action == glfw.REPEAT and key not in (glfw.KEY_UP, glfw.KEY_DOWN):
            return
        self.key_callback(key)

    def on_cursor(self, window, x, y):
        previous, self.last_cursor = self.last_cursor, (x, y)
        width, height = glfw.get_window_size(window)
        if previous is None or height <= 0 or x < width * 0.38:
            return
        if glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        else:
            return
        mujoco.mjv_moveCamera(self.model, action, (x - previous[0]) / height,
                             (y - previous[1]) / height, self.scene, self.cam)

    def on_scroll(self, window, x_offset, y_offset):
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0,
                             -0.05 * y_offset, self.scene, self.cam)

    def on_mouse_button(self, window, button, action, mods):
        if button != glfw.MOUSE_BUTTON_LEFT or action != glfw.PRESS or mods:
            return
        x, y = glfw.get_cursor_pos(window)
        for x1, y1, x2, y2, key, _ in self.buttons:
            if x1 <= x <= x2 and y1 <= y <= y2:
                self.key_callback(key)
                return

    def set_reset_status(self, status):
        self.reset_status = status

    def lock(self):
        # GLFW/MuJoCo rendering runs exclusively on the Python main thread.
        return nullcontext()

    def is_focused(self):
        return bool(glfw.get_window_attrib(self.window, glfw.FOCUSED))

    def is_running(self):
        return not glfw.window_should_close(self.window)

    def set_texts(self, texts):
        self.text = "\n".join(item[2] for item in texts if item[2])

    def set_images(self, images):
        self.images = [images] if isinstance(images, tuple) else images

    def sync(self, capture_path=None):
        glfw.poll_events()
        # Offscreen reference rendering can replace the current GL context.
        glfw.make_context_current(self.window)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self.context)
        width, height = glfw.get_framebuffer_size(self.window)
        window_width, _ = glfw.get_window_size(self.window)
        if width <= 0 or height <= 0 or window_width <= 0:
            return
        scale = width / window_width
        panel_width = round(width * 0.38)
        self.panel_width = panel_width
        mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                              mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(mujoco.MjrRect(panel_width, 0, width - panel_width, height),
                          self.scene, self.context)
        size = round(16 * scale)
        if self.font_size != size:
            self.font = ImageFont.truetype(str(self.font_path), size)
            self.font_size = size
        panel_key = (self.text, panel_width, height, scale, size, self.reset_status is not None)
        if panel_key != self.panel_key:
            self.panel_key = panel_key
            self.panel_pixels = self.draw_panel(panel_width, height, scale)
        mujoco.mjr_drawPixels(self.panel_pixels, None,
                             mujoco.MjrRect(0, 0, panel_width, height), self.context)
        for rect, image in self.images:
            mujoco.mjr_drawPixels(np.ascontiguousarray(image[::-1]).reshape(-1),
                                  None, rect, self.context)
        if capture_path is not None:
            pixels = np.empty((height, width, 3), dtype=np.uint8)
            mujoco.mjr_readPixels(pixels, None, mujoco.MjrRect(0, 0, width, height), self.context)
            Image.fromarray(pixels[::-1]).save(capture_path)
        glfw.swap_buffers(self.window)

    def draw_panel(self, panel_width, height, scale):
        panel = Image.new("RGB", (panel_width, height), (24, 29, 37))
        draw = ImageDraw.Draw(panel)
        x, y = round(16 * scale), round(18 * scale)
        line_height = round(26 * scale)
        for line in self.text.splitlines():
            for wrapped in panel_lines(line, self.font, panel_width-2*x):
                draw.text((x, y), wrapped, font=self.font, fill=(234, 239, 245))
                y += line_height
        self.buttons = []
        if self.reset_status is not None:
            window_height = height / scale
            window_panel = panel_width / scale
            button_width = (window_panel - 48) / 2
            self.buttons = [
                (16, window_height - 104, 16 + button_width, window_height - 64,
                 glfw.KEY_E, "Enable all (E)"),
                (32 + button_width, window_height - 104, window_panel - 16, window_height - 64,
                 glfw.KEY_S, "Enable joint (S)"),
                (16, window_height - 58, 16 + button_width, window_height - 18,
                 glfw.KEY_R, "Reset (R)"),
                (32 + button_width, window_height - 58, window_panel - 16, window_height - 18,
                 glfw.KEY_P, "Save rest (P)"),
            ]
            draw.rectangle((0, height - round(116 * scale), panel_width, height),
                           fill=(24, 29, 37))
            for x1, y1, x2, y2, _, label in self.buttons:
                bounds = tuple(round(value * scale) for value in (x1, y1, x2, y2))
                draw.rounded_rectangle(bounds, radius=round(5*scale),
                                       fill=(43, 69, 88), outline=(96, 140, 164))
                draw.text((round((x1 + 10) * scale), round((y1 + 9) * scale)),
                          label, font=self.font, fill=(234, 239, 245))
        return np.ascontiguousarray(np.asarray(panel)[::-1]).reshape(-1)

    def diagnostics(self):
        return {"geomgroup": self.opt.geomgroup.tolist(), "scene_geoms": self.scene.ngeom,
                "keyboard": "exclusive", "language": "en",
                "buttons": [button[-1] for button in self.buttons]}

    def close(self):
        if getattr(self, "context", None):
            glfw.make_context_current(self.window)
            self.context.free()
            self.context = None
        self.scene = None  # MjvScene releases native storage in its Python destructor.
        if self.window:
            glfw.destroy_window(self.window)
            self.window = None
        glfw.terminate()
