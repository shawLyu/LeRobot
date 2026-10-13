import pytest

from lerobot_arm_control.runtime.common import defer_cyclic_gc, next_deadline


def test_work_does_not_accumulate_into_control_period():
    deadline = 0.0
    for _ in range(100):
        deadline = next_deadline(deadline, deadline + 0.004, 0.01)
    assert deadline == pytest.approx(1.0)


def test_overrun_skips_expired_slots_without_catchup_burst():
    assert next_deadline(0.0, 0.035, 0.01) == pytest.approx(0.04)


def test_gc_restored_after_arm_hold_even_on_exception():
    import gc
    from contextlib import ExitStack

    previously_enabled = gc.isenabled()
    try:
        gc.enable()
        with pytest.raises(RuntimeError):
            with ExitStack() as stack:
                defer_cyclic_gc(stack)

                def hold():
                    assert not gc.isenabled()

                stack.callback(hold)
                assert not gc.isenabled()
                raise RuntimeError("worker failure")
        assert gc.isenabled()
        gc.disable()
        with ExitStack() as stack:
            defer_cyclic_gc(stack)
        assert not gc.isenabled()
    finally:
        if previously_enabled:
            gc.enable()
        else:
            gc.disable()
