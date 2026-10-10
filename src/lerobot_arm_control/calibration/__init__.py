"""Public calibration API; optional dependencies load on demand."""

from lerobot_arm_control.core.api import public_symbol

_EXPORTS = {
    "AngleCalibration": ("calibration.angle_calibration", "AngleCalibration"),
    "JawHeight": ("calibration.height_calibration", "JawHeight"),
    "fit_samples": ("calibration.height_calibration", "fit_samples"),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    return public_symbol(name, _EXPORTS, globals())
