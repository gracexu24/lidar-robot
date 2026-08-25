"""GPIO step/direction output for four independent stepper drivers."""

import threading
import time

try:
    from gpiozero import DigitalOutputDevice
except ImportError:
    DigitalOutputDevice = None


class StepperWheel:
    """Generate pulses for one STEP/DIR/RESET stepper driver."""

    def __init__(self, step_pin, dir_pin, reset_pin, direction_inverted=False):
        if DigitalOutputDevice is None:
            raise RuntimeError(
                'gpiozero is not installed; run: '
                'sudo apt install python3-gpiozero'
            )

        self._step = DigitalOutputDevice(step_pin, initial_value=False)
        self._direction = DigitalOutputDevice(dir_pin, initial_value=False)
        self._reset = DigitalOutputDevice(reset_pin, initial_value=False)
        self._inverted = direction_inverted
        self._rate = 0.0
        self._step_count = 0
        self._lock = threading.Lock()
        self._changed = threading.Event()
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._pulse_loop, daemon=True)
        self._reset.on()
        self._thread.start()

    def set_rate(self, signed_steps_per_second):
        """Set the logical pulse rate and direction."""
        with self._lock:
            self._rate = float(signed_steps_per_second)
        self._changed.set()

    def step_count(self):
        """Return the signed count of pulses generated."""
        with self._lock:
            return self._step_count

    def _pulse_loop(self):
        last_sign = 0
        while not self._stopping.is_set():
            with self._lock:
                rate = self._rate

            if abs(rate) < 0.01:
                self._step.off()
                self._changed.wait(timeout=0.1)
                self._changed.clear()
                last_sign = 0
                continue

            logical_sign = 1 if rate > 0.0 else -1
            if logical_sign != last_sign:
                self._step.off()
                physical_forward = (logical_sign > 0) != self._inverted
                self._direction.value = physical_forward
                time.sleep(0.00001)
                last_sign = logical_sign

            half_period = 0.5 / abs(rate)
            self._step.on()
            time.sleep(half_period)
            self._step.off()
            with self._lock:
                self._step_count += logical_sign
            time.sleep(half_period)

    def close(self):
        """Stop pulses, hold the driver in reset, and release GPIO."""
        self.set_rate(0.0)
        self._stopping.set()
        self._changed.set()
        self._thread.join(timeout=1.0)
        self._step.off()
        self._reset.off()
        self._step.close()
        self._direction.close()
        self._reset.close()


class FourWheelHardware:
    """Own and update the four GPIO wheel channels."""

    def __init__(self, step_pins, dir_pins, reset_pins, direction_inverted):
        values = (step_pins, dir_pins, reset_pins, direction_inverted)
        if any(len(value) != 4 for value in values):
            raise ValueError(
                'Each wheel parameter must contain exactly four values'
            )
        self.wheels = [
            StepperWheel(step, direction, reset, inverted)
            for step, direction, reset, inverted in zip(*values)
        ]

    def set_rates(self, rates):
        """Set logical rates in LF, LR, RF, RR order."""
        for wheel, rate in zip(self.wheels, rates):
            wheel.set_rate(rate)

    def step_counts(self):
        """Return signed pulse counts in LF, LR, RF, RR order."""
        return [wheel.step_count() for wheel in self.wheels]

    def stop(self):
        """Request zero rate on every wheel."""
        self.set_rates([0.0] * 4)

    def close(self):
        """Disable and release all wheel channels."""
        for wheel in self.wheels:
            wheel.close()
