"""Private range-file persistence; no bus access, torque changes or motion."""

import json
from pathlib import Path

from lerobot_arm_control.core.files import atomic_json


def read_ranges(path):
    if path is None or not Path(path).exists():
        return {'version': 1, 'arms': {}}
    record = json.loads(Path(path).read_text())
    if record.get('version') != 1 or not isinstance(record.get('arms'), dict):
        raise ValueError('无效的范围文件')
    return record


def save_range(path, context, joint, item):
    if path is None or not context.get('arm'):
        raise ValueError('当前连接未配置范围保存路径')
    record = read_ranges(path)
    record['arms'].setdefault(context['arm'], {})[joint] = item
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    atomic_json(path, record)
