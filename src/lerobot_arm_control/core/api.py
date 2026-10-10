"""Load public symbols when requested, keeping optional runtimes independent."""

from importlib import import_module


def public_symbol(name, exports, namespace):
    try:
        module, attribute = exports[name]
    except KeyError:
        message = f"module {namespace['__name__']!r} has no attribute {name!r}"
        raise AttributeError(message) from None
    value = getattr(import_module("lerobot_arm_control." + module), attribute)
    namespace[name] = value
    return value
