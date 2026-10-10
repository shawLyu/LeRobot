"""SO-101 control system built on the upstream LeRobot motor bus.

Importing this package does not load LeRobot, MuJoCo, OpenCV, or connect hardware.
"""

from .core.config import ArmConfig, load_arms
from .core.paths import resource_path, workspace_root
from .transport.realtime import JOINTS

__version__ = "0.2.0"
__all__ = ["ArmConfig", "JOINTS", "load_arms", "resource_path",
           "workspace_root"]
