"""STS3215-specific LeRobot construction and supported hardware configuration."""


from contextlib import contextmanager


def validate_arm(arm):
    if arm.model != 'sts3215' or arm.baudrate != 1_000_000:
        raise ValueError(f'{arm.name}: Feetech supports STS3215 at 1,000,000 baud')
    if not arm.port.startswith('/dev/serial/by-id/'):
        raise ValueError(f'{arm.name}: use a stable /dev/serial/by-id/ port')
    if any(not 1 <= motor_id <= 253 for motor_id in arm.motors.values()):
        raise ValueError(f'{arm.name}: motor IDs must be integers in 1..253')


def make_bus(arm):
    # Optional SDK imports belong to this adapter, never to configuration/model code.
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    return FeetechMotorsBus(
        port=arm.port,
        motors={name: Motor(motor_id, arm.model, MotorNormMode.RANGE_M100_100)
                for name, motor_id in arm.motors.items()},
    )


def validate_position_stream(stream, joints):
    """Validate native STS RAM streaming knobs independently of controller gains."""
    if stream is not None and (not {"acceleration", "velocity_ticks_s"} <= set(stream)
                               <= {"acceleration", "velocity_ticks_s", "locked_p_coefficients",
                                   "locked_i_coefficients",
                                   "transition_acceleration"}
                               or any(type(stream[k]) is not int for k in
                                      ("acceleration", "velocity_ticks_s"))
                               or not 1 <= stream["acceleration"] <= 254
                               or not 1 <= stream["velocity_ticks_s"] <= 3000):
        raise ValueError("invalid native streaming profile")
    if stream is not None and 'transition_acceleration' in stream:
        transition = stream['transition_acceleration']
        if type(transition) is not int or not 1 <= transition <= stream['acceleration']:
            raise ValueError('invalid native transition acceleration')
    coefficients = stream.get('locked_p_coefficients', {}) if stream else {}
    if (not isinstance(coefficients, dict) or not set(coefficients) <= (set(joints) - {'gripper'})
            or any(type(v) is not int or not 16 <= v <= 32
                   for v in coefficients.values())):
        raise ValueError('invalid temporary locked P coefficients')
    coefficients = stream.get('locked_i_coefficients', {}) if stream else {}
    if (not isinstance(coefficients, dict) or not set(coefficients) <= (set(joints) - {'gripper'})
            or any(type(v) is not int or not 0 <= v <= 16 for v in coefficients.values())):
        raise ValueError('invalid temporary locked I coefficients')


def require_sts3215(arm):
    if arm.backend != 'feetech' or arm.model != 'sts3215':
        raise ValueError('native register control requires the Feetech STS3215 backend')


@contextmanager
def connected_sts3215(arm, *, allow_alarm=False):
    """Capability gate for existing native tools and SO101 hardware controllers."""
    from lerobot_arm_control.robot.connection import connected

    require_sts3215(arm)
    with connected(arm, allow_alarm=allow_alarm) as bus:
        yield bus
