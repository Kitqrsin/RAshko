"""Strict configuration loading for the Raspberry Pi controller."""

from __future__ import annotations

import ipaddress
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from .protocol import DEFAULT_FRESHNESS_MS, JOINT_NAMES, MAX_ANGLES_DEG, MIN_ANGLES_DEG


DEFAULT_ENV_FILE = Path("/etc/rashko-controller.env")
DEFAULT_SERVO_CONFIG = Path("/etc/rashko-servos.json")
VALID_BACKENDS = frozenset({"dry-run", "pca9685"})
DEFAULT_HOME_DEGREES = (90.0, 88.0, 90.0, 96.0, 98.0, 0.0)


class ConfigError(ValueError):
    """Configuration is missing, unsafe, or internally inconsistent."""


@dataclass(frozen=True, slots=True)
class ServoCalibration:
    name: str
    channel: int
    inverted: bool
    logical_min_deg: float
    logical_max_deg: float
    actuation_range_deg: float
    min_pulse_us: int
    max_pulse_us: int

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or self.name not in JOINT_NAMES:
            raise ConfigError(f"unknown servo name: {self.name!r}")
        if (
            isinstance(self.channel, bool)
            or not isinstance(self.channel, int)
            or not 0 <= self.channel <= 15
        ):
            raise ConfigError(f"{self.name}: channel must be in 0..15")
        if not isinstance(self.inverted, bool):
            raise ConfigError(f"{self.name}: inverted must be true or false")
        numeric = (
            self.logical_min_deg,
            self.logical_max_deg,
            self.actuation_range_deg,
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in numeric
        ):
            raise ConfigError(f"{self.name}: degree values must be finite")
        if not 0.0 <= self.logical_min_deg < self.logical_max_deg:
            raise ConfigError(f"{self.name}: invalid logical degree range")
        if not self.logical_max_deg <= self.actuation_range_deg <= 300.0:
            raise ConfigError(
                f"{self.name}: actuation range must cover its logical range and be <= 300"
            )
        if (
            isinstance(self.min_pulse_us, bool)
            or isinstance(self.max_pulse_us, bool)
            or not isinstance(self.min_pulse_us, int)
            or not isinstance(self.max_pulse_us, int)
            or not 400 <= self.min_pulse_us < self.max_pulse_us <= 3000
        ):
            raise ConfigError(
                f"{self.name}: pulse range must be increasing and within 400..3000 us"
            )


@dataclass(frozen=True, slots=True)
class HardwareConfig:
    servos: tuple[ServoCalibration, ...]
    home_degrees: tuple[float, ...]
    frequency_hz: float = 50.0
    i2c_address: int = 0x40
    oe_gpio: int = 17
    max_speed_deg_per_second: float = 180.0

    def __post_init__(self) -> None:
        if len(self.servos) != len(JOINT_NAMES):
            raise ConfigError(f"expected exactly {len(JOINT_NAMES)} servo definitions")
        if tuple(servo.name for servo in self.servos) != JOINT_NAMES:
            raise ConfigError(f"servos must be ordered as {list(JOINT_NAMES)}")
        channels = tuple(servo.channel for servo in self.servos)
        if len(set(channels)) != len(channels):
            raise ConfigError("servo channels must be unique")
        if len(self.home_degrees) != len(JOINT_NAMES):
            raise ConfigError("home_degrees must contain exactly six values")
        for index, angle in enumerate(self.home_degrees):
            servo = self.servos[index]
            if (
                isinstance(angle, bool)
                or not isinstance(angle, (int, float))
                or not math.isfinite(angle)
                or not servo.logical_min_deg <= angle <= servo.logical_max_deg
            ):
                raise ConfigError(f"home_degrees[{index}] is outside the {servo.name} range")
        if (
            isinstance(self.frequency_hz, bool)
            or not isinstance(self.frequency_hz, (int, float))
            or not math.isfinite(self.frequency_hz)
            or not 40.0 <= self.frequency_hz <= 100.0
        ):
            raise ConfigError("frequency_hz must be in 40..100")
        if (
            isinstance(self.i2c_address, bool)
            or not isinstance(self.i2c_address, int)
            or not 0x03 <= self.i2c_address <= 0x77
        ):
            raise ConfigError("i2c_address must be a valid 7-bit device address")
        if (
            isinstance(self.oe_gpio, bool)
            or not isinstance(self.oe_gpio, int)
            or not 0 <= self.oe_gpio <= 27
        ):
            raise ConfigError("oe_gpio must be a BCM GPIO number in 0..27")
        if (
            isinstance(self.max_speed_deg_per_second, bool)
            or not isinstance(self.max_speed_deg_per_second, (int, float))
            or not math.isfinite(self.max_speed_deg_per_second)
            or not 1.0 <= self.max_speed_deg_per_second <= 720.0
        ):
            raise ConfigError("max_speed_deg_per_second must be in 1..720")


@dataclass(frozen=True, slots=True)
class ControllerConfig:
    bind_host: str
    port: int
    shared_secret: str
    allow_insecure: bool
    allowed_clients: frozenset[str]
    watchdog_ms: int
    freshness_ms: int
    update_hz: float
    max_sessions: int
    hardware_backend: str
    servo_config_path: Path
    hardware: HardwareConfig

    def __post_init__(self) -> None:
        try:
            bind_address = ipaddress.ip_address(self.bind_host)
        except ValueError as exc:
            raise ConfigError("RASHKO_BIND_HOST must be a literal IP address") from exc
        if bind_address.version != 4:
            raise ConfigError("the current controller supports IPv4 only")
        if (
            isinstance(self.port, bool)
            or not isinstance(self.port, int)
            or not 1 <= self.port <= 65535
        ):
            raise ConfigError("RASHKO_PI_PORT must be in 1..65535")
        if self.shared_secret and len(self.shared_secret.encode("utf-8")) < 32:
            raise ConfigError("RASHKO_SHARED_SECRET must contain at least 32 bytes")
        if not self.shared_secret and not self.allow_insecure:
            raise ConfigError(
                "RASHKO_SHARED_SECRET is required; only an explicit "
                "RASHKO_ALLOW_INSECURE=true permits unsigned packets"
            )
        if (
            isinstance(self.watchdog_ms, bool)
            or not isinstance(self.watchdog_ms, int)
            or not 100 <= self.watchdog_ms <= 10_000
        ):
            raise ConfigError("RASHKO_WATCHDOG_MS must be in 100..10000")
        if (
            isinstance(self.freshness_ms, bool)
            or not isinstance(self.freshness_ms, int)
            or not 500 <= self.freshness_ms <= 60_000
        ):
            raise ConfigError("RASHKO_FRESHNESS_MS must be in 500..60000")
        if (
            isinstance(self.update_hz, bool)
            or not isinstance(self.update_hz, (int, float))
            or not math.isfinite(self.update_hz)
            or not 10.0 <= self.update_hz <= 200.0
        ):
            raise ConfigError("RASHKO_UPDATE_HZ must be in 10..200")
        if (
            isinstance(self.max_sessions, bool)
            or not isinstance(self.max_sessions, int)
            or not 1 <= self.max_sessions <= 1024
        ):
            raise ConfigError("RASHKO_MAX_SESSIONS must be in 1..1024")
        if self.hardware_backend not in VALID_BACKENDS:
            raise ConfigError(
                f"RASHKO_HARDWARE_BACKEND must be one of {sorted(VALID_BACKENDS)}"
            )


def default_hardware_config() -> HardwareConfig:
    """Conservative legacy calibration; real hardware must use an explicit file."""

    channels = (0, 1, 2, 4, 5, 6)
    inverted = (False, True, True, False, False, False)
    pulses = (
        (1000, 2000),
        (833, 2167),
        (500, 2500),
        (500, 2500),
        (1000, 2000),
        (1000, 2000),
    )
    actuation_ranges = (270.0, 180.0, 180.0, 180.0, 180.0, 180.0)
    servos = tuple(
        ServoCalibration(
            name=name,
            channel=channels[index],
            inverted=inverted[index],
            logical_min_deg=MIN_ANGLES_DEG[index],
            logical_max_deg=MAX_ANGLES_DEG[index],
            actuation_range_deg=actuation_ranges[index],
            min_pulse_us=pulses[index][0],
            max_pulse_us=pulses[index][1],
        )
        for index, name in enumerate(JOINT_NAMES)
    )
    return HardwareConfig(servos=servos, home_degrees=DEFAULT_HOME_DEGREES)


def load_hardware_config(path: Path, *, require_file: bool) -> HardwareConfig:
    if not path.is_file():
        if require_file:
            raise ConfigError(
                f"servo calibration file {path} is required for the pca9685 backend"
            )
        return default_hardware_config()
    try:
        document = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_json_keys,
            parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)),
        )
    except (OSError, UnicodeError, ValueError) as exc:
        raise ConfigError(f"cannot read servo calibration {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise ConfigError("servo calibration root must be a JSON object")
    expected_top = {
        "frequency_hz",
        "i2c_address",
        "oe_gpio",
        "max_speed_deg_per_second",
        "home_degrees",
        "servos",
    }
    _require_json_keys(document, expected_top, "servo calibration")
    raw_servos = document["servos"]
    if not isinstance(raw_servos, list):
        raise ConfigError("servos must be an array")
    servo_fields = {
        "name",
        "channel",
        "inverted",
        "logical_min_deg",
        "logical_max_deg",
        "actuation_range_deg",
        "min_pulse_us",
        "max_pulse_us",
    }
    servos: list[ServoCalibration] = []
    for index, raw_servo in enumerate(raw_servos):
        if not isinstance(raw_servo, dict):
            raise ConfigError(f"servos[{index}] must be an object")
        _require_json_keys(raw_servo, servo_fields, f"servos[{index}]")
        try:
            servos.append(
                ServoCalibration(
                    name=raw_servo["name"],
                    channel=raw_servo["channel"],
                    inverted=raw_servo["inverted"],
                    logical_min_deg=_strict_float(
                        raw_servo["logical_min_deg"], f"servos[{index}].logical_min_deg"
                    ),
                    logical_max_deg=_strict_float(
                        raw_servo["logical_max_deg"], f"servos[{index}].logical_max_deg"
                    ),
                    actuation_range_deg=_strict_float(
                        raw_servo["actuation_range_deg"],
                        f"servos[{index}].actuation_range_deg",
                    ),
                    min_pulse_us=raw_servo["min_pulse_us"],
                    max_pulse_us=raw_servo["max_pulse_us"],
                )
            )
        except KeyError as exc:  # Kept defensive if the schema changes.
            raise ConfigError(f"servos[{index}] is missing {exc.args[0]}") from exc
    raw_home = document["home_degrees"]
    if not isinstance(raw_home, list):
        raise ConfigError("home_degrees must be an array")
    home = tuple(_strict_float(value, "home_degrees") for value in raw_home)
    return HardwareConfig(
        servos=tuple(servos),
        home_degrees=home,
        frequency_hz=_strict_float(document["frequency_hz"], "frequency_hz"),
        i2c_address=_strict_int(document["i2c_address"], "i2c_address"),
        oe_gpio=_strict_int(document["oe_gpio"], "oe_gpio"),
        max_speed_deg_per_second=_strict_float(
            document["max_speed_deg_per_second"], "max_speed_deg_per_second"
        ),
    )


def load_controller_config(
    environ: Mapping[str, str] | None = None,
    *,
    env_file: Path | None = None,
) -> ControllerConfig:
    source_environment = os.environ if environ is None else environ
    selected_env_file = env_file
    if selected_env_file is None:
        selected_env_file = Path(
            source_environment.get("RASHKO_ENV_FILE", str(DEFAULT_ENV_FILE))
        )
    values = _read_env_file(selected_env_file)
    values.update(source_environment)

    backend = values.get("RASHKO_HARDWARE_BACKEND", "dry-run").strip().lower()
    servo_path = Path(values.get("RASHKO_SERVO_CONFIG", str(DEFAULT_SERVO_CONFIG)))
    allowed_clients = _parse_allowed_clients(values.get("RASHKO_ALLOWED_CLIENTS", ""))
    allow_insecure = _parse_bool(
        values.get("RASHKO_ALLOW_INSECURE", "false"),
        "RASHKO_ALLOW_INSECURE",
    )
    hardware = load_hardware_config(
        servo_path,
        require_file=backend == "pca9685",
    )
    return ControllerConfig(
        bind_host=values.get("RASHKO_BIND_HOST", "0.0.0.0").strip(),
        port=_parse_int(values.get("RASHKO_PI_PORT", "5005"), "RASHKO_PI_PORT"),
        shared_secret=values.get("RASHKO_SHARED_SECRET", ""),
        allow_insecure=allow_insecure,
        allowed_clients=allowed_clients,
        watchdog_ms=_parse_int(
            values.get("RASHKO_WATCHDOG_MS", "500"), "RASHKO_WATCHDOG_MS"
        ),
        freshness_ms=_parse_int(
            values.get("RASHKO_FRESHNESS_MS", str(DEFAULT_FRESHNESS_MS)),
            "RASHKO_FRESHNESS_MS",
        ),
        update_hz=_parse_float(
            values.get("RASHKO_UPDATE_HZ", "50"), "RASHKO_UPDATE_HZ"
        ),
        max_sessions=_parse_int(
            values.get("RASHKO_MAX_SESSIONS", "128"), "RASHKO_MAX_SESSIONS"
        ),
        hardware_backend=backend,
        servo_config_path=servo_path,
        hardware=hardware,
    )


def _read_env_file(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as exc:
        raise ConfigError(f"cannot read environment file {path}: {exc}") from exc
    result: dict[str, str] = {}
    for line_number, original in enumerate(lines, start=1):
        line = original.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].lstrip()
        if "=" not in line:
            raise ConfigError(f"{path}:{line_number}: expected KEY=value")
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key or not key.replace("_", "").isalnum() or not key[0].isalpha():
            raise ConfigError(f"{path}:{line_number}: invalid environment variable name")
        if key in result:
            raise ConfigError(f"{path}:{line_number}: duplicate setting {key}")
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
            value = value[1:-1]
        result[key] = value
    return result


def _parse_allowed_clients(raw: str) -> frozenset[str]:
    result: set[str] = set()
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        try:
            address = ipaddress.ip_address(item)
        except ValueError as exc:
            raise ConfigError(f"invalid RASHKO_ALLOWED_CLIENTS address: {item}") from exc
        if address.version != 4:
            raise ConfigError("RASHKO_ALLOWED_CLIENTS currently supports IPv4 only")
        result.add(str(address))
    return frozenset(result)


def _parse_bool(raw: str, name: str) -> bool:
    normalized = raw.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ConfigError(f"{name} must be true or false")


def _parse_int(raw: str, name: str) -> int:
    try:
        return int(raw, 10)
    except ValueError as exc:
        raise ConfigError(f"{name} must be an integer") from exc


def _parse_float(raw: str, name: str) -> float:
    try:
        result = float(raw)
    except ValueError as exc:
        raise ConfigError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ConfigError(f"{name} must be finite")
    return result


def _strict_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigError(f"{name} must be finite")
    return result


def _strict_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{name} must be an integer")
    return value


def _require_json_keys(document: Mapping[str, object], expected: set[str], name: str) -> None:
    keys = set(document)
    missing = expected - keys
    unexpected = keys - expected
    if missing:
        raise ConfigError(f"{name} is missing fields: {sorted(missing)}")
    if unexpected:
        raise ConfigError(f"{name} has unknown fields: {sorted(unexpected)}")


def _reject_duplicate_json_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result
