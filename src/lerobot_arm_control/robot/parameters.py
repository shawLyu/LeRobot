"""Verify the saved STS baseline and restore differences without enabling motion."""

import hashlib
import logging
import time

from lerobot_arm_control.core.files import atomic_json
from lerobot_arm_control.core.paths import workspace_root
from lerobot_arm_control.workbench.hardware import ServoAccess
from lerobot_arm_control.workbench.registers import BAUDRATES, BY_KEY, REGISTERS, install_table

LOGGER = logging.getLogger(__name__)
STARTUP_KEYS = {'Acceleration', 'Goal_Time', 'Goal_Velocity', 'Torque_Limit'}
PERSISTENT_KEYS = {r.key for r in REGISTERS if r.group == 'EEPROM' and r.effect != 'transport'}
IDENTITY_KEYS = {r.key for r in REGISTERS
                 if r.group in ('Identity', 'Factory') or r.effect == 'transport'}


def validate_parameters(arm):
    """Reject incomplete or unsafe snapshots before the first hardware write."""
    for joint, profile in arm.parameters.items():
        if set(profile) != {'persistent', 'startup', 'identity'}:
            raise ValueError(f'{joint}: invalid hardware profile sections')
        for group, keys in (('persistent', PERSISTENT_KEYS), ('startup', STARTUP_KEYS),
                            ('identity', IDENTITY_KEYS)):
            if set(profile[group]) != keys:
                raise ValueError(f'{joint}: incomplete {group} parameter baseline')
            for key, value in profile[group].items():
                if type(value) is not int:
                    raise ValueError(f'{joint}/{key}: expected integer')
                if group != 'identity':
                    BY_KEY[key].check(value)
        saved = profile['persistent']
        if (saved['Min_Position_Limit'] >= saved['Max_Position_Limit']
                or saved['Min_Voltage_Limit'] >= saved['Max_Voltage_Limit']
                or saved['Operating_Mode'] != 0):
            raise ValueError(f'{joint}: invalid position/voltage range or non-position mode')
        identity = profile['identity']
        if (identity['ID'] != arm.motors[joint]
                or identity['Model_Number'] != 777
                or not 0 <= identity['Baud_Rate'] < len(BAUDRATES)
                or BAUDRATES[identity['Baud_Rate']] != arm.baudrate):
            raise ValueError(f'{joint}: saved transport/model does not match the arm')
        if profile['startup']['Torque_Limit'] > saved['Max_Torque_Limit']:
            raise ValueError(f'{joint}: startup output exceeds the saved maximum')
        if profile['startup']['Goal_Time'] or profile['startup']['Goal_Velocity']:
            raise ValueError(f'{joint}: startup time/velocity targets must be zero')


def activation_marker(arm):
    identity = hashlib.sha256((arm.name+'|'+arm.port).encode()).hexdigest()[:20]
    return workspace_root()/'logs/parameter-activation'/f'{identity}.json'


def confirm_power_cycle(arm):
    """Explicit operator confirmation for Phase bits without a reboot counter."""
    from dataclasses import replace

    from lerobot_arm_control.robot.connection import connected

    marker = activation_marker(arm)
    if not marker.exists():
        raise ValueError(f'{arm.name}: no pending Phase restart')
    with connected(replace(arm, parameters=None), allow_alarm=True) as bus:
        install_table(bus)
        access = ServoAccess(bus)
        for joint, profile in arm.parameters.items():
            if (access.read('Phase', joint) != profile['persistent']['Phase']
                    or access.read('Torque_Enable', joint) or access.read('Moving', joint)):
                raise RuntimeError('confirm requires saved Phase and released, stationary motors')
        marker.unlink()
        try:
            return ensure_parameters(bus, arm)
        except BaseException:
            atomic_json(marker, {'arm': arm.name, 'requires_power_cycle': True})
            raise


def ensure_parameters(bus, arm):
    """Call while holding the exclusive serial lock, before exposing the bus.

    An unchanged baseline performs no writes. Restoration requires every online
    joint to be released and stationary; this function never drops support from
    a loaded arm. Runtime controllers exclusively own temporary gain overrides.
    """
    validate_parameters(arm)
    marker = activation_marker(arm)
    if marker.exists():
        raise RuntimeError(f'{arm.name}: Phase restart is pending; power-cycle the arm, then run '
                           f'config confirm-power-cycle --arm {arm.name}')
    install_table(bus)
    access = ServoAccess(bus)
    online = [j for j in arm.motors if j in bus.motors]
    report = {'arm': arm.name, 'checked': online,
              'missing': [j for j in arm.motors if j not in bus.motors], 'restored': []}
    changes = []
    for joint in online:
        profile = arm.parameters[joint]
        for key, expected in profile['identity'].items():
            actual = access.read(key, joint)
            if actual != expected:
                raise RuntimeError(f'{arm.name}/{joint}/{key}: identity/factory mismatch '
                                   f'{actual} != {expected}; refusing to configure another device')
        for group in ('persistent', 'startup'):
            for key, expected in profile[group].items():
                actual = access.read(key, joint)
                if actual != expected:
                    changes.append((joint, key, actual, expected))
        if access.read('Lock', joint) != 1:
            raise RuntimeError(f'{arm.name}/{joint}: EEPROM is unlocked; '
                               'finish its transaction first')
    if not changes:
        return report

    def released_and_healthy():
        for joint in online:
            if access.read('Torque_Enable', joint) != 0:
                raise RuntimeError(f'{arm.name}: saved parameters differ, but torque is enabled; '
                                   'support the arm and release torque before restore')
            if access.read('Moving', joint) or access.read('Status', joint) & ~1:
                raise RuntimeError(f'{arm.name}/{joint}: moving or hardware fault; restore blocked')
            temperature = access.read('Present_Temperature', joint)
            voltage = access.read('Present_Voltage', joint)
            saved = arm.parameters[joint]['persistent']
            if (temperature >= saved['Max_Temperature_Limit']
                    or not saved['Min_Voltage_Limit'] <= voltage <= saved['Max_Voltage_Limit']):
                raise RuntimeError(f'{arm.name}/{joint}: unsafe temperature/voltage '
                                   'for EEPROM write')

    released_and_healthy()
    positions = {j: access.read('Present_Position', j) for j in online}
    time.sleep(.03)
    released_and_healthy()
    if any(abs(access.read('Present_Position', j)-v) > 2 for j, v in positions.items()):
        raise RuntimeError(f'{arm.name}: arm is being moved; restore blocked')

    def verified_write(joint, key, value):
        # Status level zero suppresses write ACKs; a successful readback is authoritative.
        error = None
        try:
            access.write(key, joint, value)
        except ConnectionError as exc:
            error = exc
        if BY_KEY[key].group == 'EEPROM':
            time.sleep(.02)
        if access.read(key, joint) != value:
            raise RuntimeError(f'{arm.name}/{joint}/{key}: write verification failed') from error

    def write_order(change):
        _, key, before, expected = change
        # Expand paired bounds before contracting the opposite end. Enable write
        # replies first; suppress them only after other EEPROM fields are saved.
        priority = 0
        if key == 'Response_Status_Level':
            priority = -3 if expected else 2
        elif key == 'Operating_Mode':
            priority = -2
        elif ((key.startswith('Min_') and expected < before)
              or (key.startswith('Max_') and expected > before)):
            priority = -1
        return BY_KEY[key].group != 'EEPROM', priority

    changes.sort(key=write_order)
    touched, attempted = set(), []
    try:
        for joint, key, before, expected in changes:
            released_and_healthy()
            if BY_KEY[key].group == 'EEPROM' and joint not in touched:
                touched.add(joint)
                verified_write(joint, 'Lock', 0)
            attempted.append((joint, key, before, expected))
            if key == 'Phase':
                marker.parent.mkdir(parents=True, exist_ok=True)
                atomic_json(marker, {'arm': arm.name, 'requires_power_cycle': True})
            verified_write(joint, key, expected)
            report['restored'].append({'joint': joint, 'register': key,
                                       'before': before, 'after': expected})
    except BaseException as exc:
        errors = []
        for joint, key, before, _ in reversed(attempted):
            try:
                released_and_healthy()
                verified_write(joint, key, before)
            except Exception as rollback_error:
                errors.append(str(rollback_error))
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            if errors:
                LOGGER.error('%s: interrupted restore rollback errors: %s', arm.name, errors)
            raise
        raise RuntimeError(f'{arm.name}: parameter restore failed: {exc}; '
                           f'rollback errors: {errors}') from exc
    finally:
        lock_errors = []
        for joint in touched:
            try:
                verified_write(joint, 'Lock', 1)
            except Exception as exc:
                lock_errors.append(f'{joint}: {exc}')
        if lock_errors:
            raise RuntimeError(f'EEPROM relock failed: {lock_errors}')
    LOGGER.warning('%s: restored saved parameters: %s', arm.name, report['restored'])
    if any(key == 'Phase' for _, key, _, _ in changes):
        raise RuntimeError(f'{arm.name}: Phase restored; power-cycle before starting a controller')
    return report


def capture_parameters(bus, arm):
    """Explicitly save configurable values and identity, excluding goals and torque."""
    install_table(bus)
    access = ServoAccess(bus)
    if set(bus.motors) != set(arm.motors):
        raise ValueError('capture requires every configured motor online')
    parameters = {}
    for joint in arm.motors:
        if access.read('Moving', joint) or access.read('Lock', joint) != 1:
            raise RuntimeError(f'{joint}: stop motion and finish parameter '
                               'transactions before saving')
        parameters[joint] = {group: {key: access.read(key, joint) for key in sorted(keys)}
                             for group, keys in (('persistent', PERSISTENT_KEYS),
                                                 ('startup', STARTUP_KEYS),
                                                 ('identity', IDENTITY_KEYS))}
    return parameters
