"""Fail-closed servo hardware abstraction.

The PCA9685 implementation never lowers OE during construction.  PWM values may
be prepared while OE is HIGH, but only :class:`ArmController` can explicitly
enable outputs after an authenticated, fresh, valid command.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections.abc import Sequence

from .config import HardwareConfig, ServoCalibration


class HardwareError(RuntimeError):
    """The hardware could not be kept in a known controllable state."""


class ServoHardware(ABC):
    @property
    @abstractmethod
    def ready(self) -> bool:
        """Whether the driver is healthy enough to accept commands."""

    @property
    @abstractmethod
    def outputs_enabled(self) -> bool:
        """The driver's commanded OE state (not electrical feedback)."""

    @abstractmethod
    def write_angles(self, angles_deg: Sequence[float]) -> None:
        """Update PWM registers without changing OE."""

    @abstractmethod
    def enable_outputs(self) -> None:
        """Drive active-low OE LOW."""

    @abstractmethod
    def disable_outputs(self) -> None:
        """Drive active-low OE HIGH."""

    @abstractmethod
    def close(self) -> None:
        """Disable outputs and release resources."""


class DryRunHardware(ServoHardware):
    """In-memory backend for development and safety tests."""

    def __init__(self, config: HardwareConfig) -> None:
        self.config = config
        self.angles_deg = tuple(config.home_degrees)
        self.write_count = 0
        self._ready = True
        self._outputs_enabled = False

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def outputs_enabled(self) -> bool:
        return self._outputs_enabled

    def write_angles(self, angles_deg: Sequence[float]) -> None:
        if not self._ready:
            raise HardwareError("dry-run hardware is closed")
        self.angles_deg = _validate_calibrated_angles(self.config, angles_deg)
        self.write_count += 1

    def enable_outputs(self) -> None:
        if not self._ready:
            raise HardwareError("dry-run hardware is closed")
        self._outputs_enabled = True

    def disable_outputs(self) -> None:
        self._outputs_enabled = False

    def close(self) -> None:
        self.disable_outputs()
        self._ready = False


class Pca9685Hardware(ServoHardware):
    """PCA9685 PWM plus GPIO17 active-low OE, with lazy Pi-only imports."""

    def __init__(self, config: HardwareConfig) -> None:
        self.config = config
        self._ready = False
        self._outputs_enabled = False
        self._oe = None
        self._i2c = None
        self._pca = None
        try:
            from gpiozero import DigitalOutputDevice

            # active_high=True means .on() is an electrical HIGH.  Constructing
            # this first with initial_value=True is the central startup invariant.
            self._oe = DigitalOutputDevice(
                config.oe_gpio,
                active_high=True,
                initial_value=True,
            )
            import board
            import busio
            from adafruit_pca9685 import PCA9685

            self._i2c = busio.I2C(board.SCL, board.SDA)
            self._pca = PCA9685(self._i2c, address=config.i2c_address)
            self._pca.frequency = config.frequency_hz
            self._ready = True
        except Exception as exc:
            self._best_effort_disable()
            self._release_resources()
            raise HardwareError(f"failed to initialize PCA9685 hardware: {exc}") from exc

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def outputs_enabled(self) -> bool:
        return self._outputs_enabled

    def write_angles(self, angles_deg: Sequence[float]) -> None:
        if not self._ready or self._pca is None:
            raise HardwareError("PCA9685 hardware is not ready")
        angles = _validate_calibrated_angles(self.config, angles_deg)
        try:
            for servo, angle in zip(self.config.servos, angles):
                pulse_us = angle_to_pulse_us(servo, angle)
                duty_cycle = round(
                    pulse_us * self.config.frequency_hz * 65535.0 / 1_000_000.0
                )
                self._pca.channels[servo.channel].duty_cycle = max(
                    0, min(65535, duty_cycle)
                )
        except Exception as exc:
            self._ready = False
            self._best_effort_disable()
            raise HardwareError(f"failed to write PCA9685 channels: {exc}") from exc

    def enable_outputs(self) -> None:
        if not self._ready or self._oe is None:
            raise HardwareError("cannot enable outputs: hardware is not ready")
        try:
            self._oe.off()  # Electrical LOW: active-low OE enables PWM outputs.
            self._outputs_enabled = True
        except Exception as exc:
            self._ready = False
            self._best_effort_disable()
            raise HardwareError(f"failed to enable servo outputs: {exc}") from exc

    def disable_outputs(self) -> None:
        if self._oe is None:
            self._outputs_enabled = False
            return
        try:
            self._oe.on()  # Electrical HIGH: PCA9685 outputs disabled.
            self._outputs_enabled = False
        except Exception as exc:
            self._ready = False
            self._outputs_enabled = False
            raise HardwareError(f"failed to drive OE HIGH: {exc}") from exc

    def close(self) -> None:
        disable_error: Exception | None = None
        try:
            self.disable_outputs()
        except Exception as exc:  # Release resources, but report the safety failure.
            disable_error = exc
        self._ready = False
        self._release_resources()
        if disable_error is not None:
            raise HardwareError(str(disable_error)) from disable_error

    def _best_effort_disable(self) -> None:
        self._outputs_enabled = False
        if self._oe is not None:
            try:
                self._oe.on()
            except Exception:
                pass

    def _release_resources(self) -> None:
        if self._pca is not None:
            try:
                self._pca.deinit()
            except Exception:
                pass
            self._pca = None
        if self._i2c is not None:
            try:
                self._i2c.deinit()
            except Exception:
                pass
            self._i2c = None
        if self._oe is not None:
            try:
                self._oe.close()
            except Exception:
                pass
            self._oe = None


def angle_to_pulse_us(servo: ServoCalibration, logical_angle_deg: float) -> float:
    """Map a logical degree to a calibrated pulse without silently clamping."""

    angle = float(logical_angle_deg)
    if not math.isfinite(angle) or not servo.logical_min_deg <= angle <= servo.logical_max_deg:
        raise ValueError(
            f"{servo.name}={angle} is outside "
            f"{servo.logical_min_deg}..{servo.logical_max_deg} degrees"
        )
    physical_angle = servo.actuation_range_deg - angle if servo.inverted else angle
    fraction = physical_angle / servo.actuation_range_deg
    return servo.min_pulse_us + fraction * (servo.max_pulse_us - servo.min_pulse_us)


def create_hardware(backend: str, config: HardwareConfig) -> ServoHardware:
    if backend == "dry-run":
        return DryRunHardware(config)
    if backend == "pca9685":
        return Pca9685Hardware(config)
    raise ValueError(f"unsupported hardware backend: {backend}")


def _validate_calibrated_angles(
    config: HardwareConfig,
    values: Sequence[float],
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)) or len(values) != len(config.servos):
        raise ValueError(f"expected exactly {len(config.servos)} angles")
    result: list[float] = []
    for servo, raw_angle in zip(config.servos, values):
        if isinstance(raw_angle, bool):
            raise ValueError(f"{servo.name} angle must be numeric")
        try:
            angle = float(raw_angle)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{servo.name} angle must be numeric") from exc
        if not math.isfinite(angle):
            raise ValueError(f"{servo.name} angle must be finite")
        if not servo.logical_min_deg <= angle <= servo.logical_max_deg:
            raise ValueError(
                f"{servo.name}={angle} is outside "
                f"{servo.logical_min_deg}..{servo.logical_max_deg} degrees"
            )
        result.append(angle)
    return tuple(result)

