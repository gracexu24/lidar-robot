"""Tests for GPIO driver sleep and reset behavior."""

import time

from my_robot import stepper_hardware


class FakeOutput:
    """Record output transitions without accessing GPIO."""

    instances = {}

    def __init__(self, pin, initial_value=False):
        self.pin = pin
        self.history = [bool(initial_value)]
        self.closed = False
        FakeOutput.instances[pin] = self

    @property
    def value(self):
        return self.history[-1]

    @value.setter
    def value(self, new_value):
        self.history.append(bool(new_value))

    def on(self):
        self.value = True

    def off(self):
        self.value = False

    def close(self):
        self.closed = True


def wait_for(predicate, timeout=0.5):
    """Wait for a pulse-thread state change."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.001)
    raise AssertionError('Timed out waiting for motor output transition')


def test_sleep_stops_driver_without_reasserting_reset(monkeypatch):
    """Use SLEEP for routine stops and RESET only during initialization."""
    FakeOutput.instances = {}
    monkeypatch.setattr(
        stepper_hardware, 'DigitalOutputDevice', FakeOutput
    )
    debug_messages = []
    wheel = stepper_hardware.StepperWheel(
        14, 15, 4, 12, wheel_name='LF',
        debug_callback=debug_messages.append,
    )
    reset = FakeOutput.instances[4]
    sleep = FakeOutput.instances[12]

    assert reset.history == [False, True]
    assert sleep.value is False

    wheel.set_rate(100.0)
    wait_for(lambda: sleep.value is True)
    wheel.set_rate(0.0)
    wait_for(lambda: sleep.value is False)

    assert reset.history == [False, True]
    wheel.close()
    assert sleep.value is False
    assert reset.history == [False, True]
    assert debug_messages == [
        'LF: startup reset complete; driver sleeping',
        'LF: SLEEP high; driver awake',
        'LF: SLEEP low; driver disabled',
    ]
