"""Shared measured controller settings and recorded native LeRobot defaults."""

import hashlib
import json

from lerobot_arm_control.core.paths import resource_path

CONTROL_CONFIG = resource_path("configs/control.json")
CONFIG_BYTES = CONTROL_CONFIG.read_bytes()
CONTROL_CONFIG_SHA256 = hashlib.sha256(CONFIG_BYTES).hexdigest()
CONTROL = json.loads(CONFIG_BYTES)
if CONTROL.get("version") != 1:
    raise ValueError("invalid shared control configuration")
DEFAULTS = CONTROL["native_defaults"]
POSITION_FEEDBACK = CONTROL["position_feedback"]
TRAJECTORY = CONTROL["trajectory"]
SIMULATION_ACTUATOR = CONTROL["simulation_actuator"]
CONTROL_HZ = DEFAULTS["control_hz"]
READ_RETRIES = DEFAULTS["num_read_retries"]
