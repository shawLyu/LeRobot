from pathlib import Path

import glfw
import mujoco
import numpy as np
from PIL import ImageFont

from arm_control.control_viewer import ControlViewer, panel_lines
from arm_control.ui import motor_fault_text


def viewer_without_window():
    viewer = ControlViewer.__new__(ControlViewer)
    received = []
    viewer.key_callback = received.append
    viewer.opt = mujoco.MjvOption()
    viewer.opt.geomgroup[:] = (1, 1, 1, 0, 0, 0)
    return viewer, received


def test_numeric_selection_never_changes_geometry_groups():
    viewer, received = viewer_without_window()
    original = viewer.opt.geomgroup.copy()
    for key in range(glfw.KEY_1, glfw.KEY_6 + 1):
        viewer.on_key(None, key, 0, glfw.PRESS, 0)
        viewer.on_key(None, key, 0, glfw.RELEASE, 0)
        assert (viewer.opt.geomgroup == original).all()
    assert received == list(range(glfw.KEY_1, glfw.KEY_6 + 1))


def test_actual_so101_meshes_remain_identical_after_all_selection_keys():
    model = mujoco.MjModel.from_xml_path(str(
        Path(__file__).resolve().parents[1] / "assets/so101/scene_hardware.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    scene = mujoco.MjvScene(model, maxgeom=1000)
    viewer, _ = viewer_without_window()

    def geometry():
        mujoco.mjv_updateScene(model, data, viewer.opt, None, mujoco.MjvCamera(),
                              mujoco.mjtCatBit.mjCAT_ALL, scene)
        return [(scene.geoms[i].objid, scene.geoms[i].type) for i in range(scene.ngeom)]

    baseline = geometry()
    assert sum(kind == mujoco.mjtGeom.mjGEOM_MESH for _, kind in baseline) >= 15
    for key in range(glfw.KEY_1, glfw.KEY_6 + 1):
        viewer.on_key(None, key, 0, glfw.PRESS, 0)
        assert geometry() == baseline


def test_repeated_enable_does_not_reset_target_but_arrows_repeat():
    viewer, received = viewer_without_window()
    viewer.on_key(None, glfw.KEY_E, 0, glfw.PRESS, 0)
    viewer.on_key(None, glfw.KEY_E, 0, glfw.REPEAT, 0)
    viewer.on_key(None, glfw.KEY_UP, 0, glfw.REPEAT, 0)
    viewer.on_key(None, glfw.KEY_DOWN, 0, glfw.REPEAT, 0)
    assert received == [glfw.KEY_E, glfw.KEY_UP, glfw.KEY_DOWN]


def test_modified_shortcuts_do_not_trigger_motor_commands():
    viewer, received = viewer_without_window()
    viewer.on_key(None, glfw.KEY_E, 0, glfw.PRESS, glfw.MOD_CONTROL)
    viewer.on_key(None, glfw.KEY_X, 0, glfw.PRESS, glfw.MOD_SHIFT)
    assert received == []


def test_voltage_fault_is_explicit_instead_of_only_a_number():
    assert motor_fault_text(1) == "input voltage error"
    assert motor_fault_text(5) == "input voltage error, overheating"


def test_rest_buttons_route_clicks_to_same_keyboard_commands(monkeypatch):
    viewer, received = viewer_without_window()
    viewer.buttons = [(16, 742, 220, 782, glfw.KEY_R, "Reset (R)"),
                      (236, 742, 440, 782, glfw.KEY_P, "Save rest (P)")]
    for cursor in ((100, 760), (300, 760), (600, 760)):
        monkeypatch.setattr(glfw, "get_cursor_pos", lambda window: cursor)
        viewer.on_mouse_button(None, glfw.MOUSE_BUTTON_LEFT, glfw.PRESS, 0)
    viewer.on_key(None, glfw.KEY_R, 0, glfw.REPEAT, 0)
    assert received == [glfw.KEY_R, glfw.KEY_P]


def test_panel_layout_fits_words_and_long_tokens_and_caches_unchanged_lines():
    font = ImageFont.load_default(size=16)
    width = 130
    for text in ('', 'Position error: 100.21 mm', 'x'*80):
        lines = panel_lines(text, font, width)
        assert all(font.getlength(line) <= width for line in lines)
        assert ''.join(lines).replace(' ', '') == text.replace(' ', '')
    hits = panel_lines.cache_info().hits
    panel_lines('Position error: 100.21 mm', font, width)
    assert panel_lines.cache_info().hits == hits+1


def test_panel_keeps_reset_button_geometry_and_rgb_buffer_shape():
    viewer, _ = viewer_without_window()
    viewer.font = ImageFont.load_default(size=16)
    viewer.text = 'Local simulation\nReset available'
    viewer.reset_status = 'ready'
    pixels = viewer.draw_panel(486, 800, 1)
    assert pixels.shape == (486*800*3,)
    assert pixels.dtype == np.uint8 and pixels.flags.c_contiguous
    assert [button[-1] for button in viewer.buttons] == [
        'Enable all (E)', 'Enable joint (S)', 'Reset (R)', 'Save rest (P)']
