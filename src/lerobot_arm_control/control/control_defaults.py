"""Shared measured controller settings and recorded native LeRobot defaults."""

import hashlib
import json
from copy import deepcopy

from lerobot_arm_control.core.paths import resource_path

CONTROL_CONFIG = resource_path("configs/control.json")
CONFIG_BYTES = CONTROL_CONFIG.read_bytes()
CONTROL_CONFIG_SHA256 = hashlib.sha256(CONFIG_BYTES).hexdigest()
CONTROL = json.loads(CONFIG_BYTES)
if CONTROL.get("version") != 1:
    raise ValueError("invalid shared control configuration")
DEFAULTS = CONTROL["native_defaults"]
POSITION_FEEDBACK = CONTROL["position_feedback"]
CIRCLE_POSITION_FEEDBACK = deepcopy(POSITION_FEEDBACK)
circle_settings = CONTROL['circle_position_feedback']
CIRCLE_POSITION_FEEDBACK['name'] = circle_settings['name']
CIRCLE_POSITION_FEEDBACK['native_stream'].update(circle_settings['native_stream'])
for joint, settings in circle_settings['joints'].items():
    CIRCLE_POSITION_FEEDBACK['joints'][joint].update(settings)
CIRCLE90_POSITION_FEEDBACK = deepcopy(CIRCLE_POSITION_FEEDBACK)
CIRCLE90_POSITION_FEEDBACK['name'] = CONTROL['circle90_position_feedback']['name']
CIRCLE90_POSITION_FEEDBACK['native_stream'].update(
    CONTROL['circle90_position_feedback']['native_stream'])
CIRCLE_FEEDBACK_PROFILES = {p['name']: p for p in (
    CIRCLE_POSITION_FEEDBACK, CIRCLE90_POSITION_FEEDBACK)}
TRAJECTORY = CONTROL["trajectory"]
SIMULATION_ACTUATOR = CONTROL["simulation_actuator"]
CONTROL_HZ = DEFAULTS["control_hz"]
READ_RETRIES = DEFAULTS["num_read_retries"]
