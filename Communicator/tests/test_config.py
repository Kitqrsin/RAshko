from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from rashko_communicator.config import (
    ConfigError,
    ControllerConfig,
    default_hardware_config,
    is_client_allowed,
    load_controller_config,
    load_hardware_config,
)
from rashko_communicator.hardware import angle_to_pulse_us


SECRET = "c" * 64


class ConfigTests(unittest.TestCase):
    def test_authentication_is_required_by_default(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ConfigError, "SHARED_SECRET"):
                load_controller_config(
                    {"RASHKO_ENV_FILE": str(Path(directory) / "missing.env")}
                )

    def test_dry_run_uses_safe_defaults_but_hardware_requires_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "missing.json"
            config = load_controller_config(
                {
                    "RASHKO_ENV_FILE": str(Path(directory) / "missing.env"),
                    "RASHKO_SHARED_SECRET": SECRET,
                    "RASHKO_HARDWARE_BACKEND": "dry-run",
                    "RASHKO_SERVO_CONFIG": str(missing),
                }
            )
            self.assertEqual(config.hardware.servos[0].channel, 0)
            self.assertEqual(config.hardware.servos[3].channel, 4)
            with self.assertRaisesRegex(ConfigError, "required"):
                load_controller_config(
                    {
                        "RASHKO_ENV_FILE": str(Path(directory) / "missing.env"),
                        "RASHKO_SHARED_SECRET": SECRET,
                        "RASHKO_HARDWARE_BACKEND": "pca9685",
                        "RASHKO_SERVO_CONFIG": str(missing),
                    }
                )

    def test_checked_in_calibration_matches_deployed_mapping_and_home(self) -> None:
        path = Path(__file__).parents[1] / "config" / "rashko-servos.json"
        config = load_hardware_config(path, require_file=True)
        expected_home = (90.0, 88.0, 90.0, 96.0, 98.0, 0.0)
        self.assertEqual(config.home_degrees, expected_home)
        self.assertEqual(default_hardware_config().home_degrees, expected_home)
        self.assertEqual(tuple(servo.channel for servo in config.servos), (0, 1, 2, 4, 5, 6))
        self.assertEqual(
            tuple(servo.inverted for servo in config.servos),
            (False, True, True, False, False, False),
        )
        self.assertEqual(
            tuple((servo.min_pulse_us, servo.max_pulse_us) for servo in config.servos),
            (
                (1000, 2000),
                (833, 2167),
                (500, 2500),
                (500, 2500),
                (1000, 2000),
                (1000, 2000),
            ),
        )
        self.assertEqual(config.servos[0].actuation_range_deg, 270)

    def test_duplicate_channels_and_unsafe_pulses_are_rejected(self) -> None:
        base = default_hardware_config()
        document = {
            "frequency_hz": base.frequency_hz,
            "i2c_address": base.i2c_address,
            "oe_gpio": base.oe_gpio,
            "max_speed_deg_per_second": base.max_speed_deg_per_second,
            "home_degrees": list(base.home_degrees),
            "servos": [
                {
                    "name": servo.name,
                    "channel": servo.channel,
                    "inverted": servo.inverted,
                    "logical_min_deg": servo.logical_min_deg,
                    "logical_max_deg": servo.logical_max_deg,
                    "actuation_range_deg": servo.actuation_range_deg,
                    "min_pulse_us": servo.min_pulse_us,
                    "max_pulse_us": servo.max_pulse_us,
                }
                for servo in base.servos
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "servos.json"
            document["servos"][1]["channel"] = document["servos"][0]["channel"]
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ConfigError, "unique"):
                load_hardware_config(path, require_file=True)

            document["servos"][1]["channel"] = 1
            document["servos"][0]["min_pulse_us"] = 100
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ConfigError, "pulse"):
                load_hardware_config(path, require_file=True)

    def test_duplicate_json_fields_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "servos.json"
            path.write_text('{"frequency_hz":50,"frequency_hz":60}', encoding="utf-8")
            with self.assertRaisesRegex(ConfigError, "duplicate JSON key"):
                load_hardware_config(path, require_file=True)

    def test_servo_mapping_respects_inversion_without_clamping(self) -> None:
        config = default_hardware_config()
        base, shoulder = config.servos[:2]
        self.assertAlmostEqual(angle_to_pulse_us(base, 0), 1000)
        self.assertAlmostEqual(
            angle_to_pulse_us(base, 245),
            1000 + (245 / 270) * 1000,
        )
        self.assertAlmostEqual(angle_to_pulse_us(shoulder, 0), 2167)
        self.assertAlmostEqual(angle_to_pulse_us(shoulder, 180), 833)
        with self.assertRaises(ValueError):
            angle_to_pulse_us(base, 246)

    def test_allowlist_and_environment_file_are_parsed_strictly(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            env_path = Path(directory) / "controller.env"
            env_path.write_text(
                "\n".join(
                    (
                        f"RASHKO_SHARED_SECRET={SECRET}",
                        "RASHKO_ALLOWED_CLIENTS=192.168.1.10,192.168.1.11",
                        "RASHKO_HARDWARE_BACKEND=dry-run",
                    )
                ),
                encoding="utf-8",
            )
            config = load_controller_config({}, env_file=env_path)
            self.assertEqual(
                config.allowed_clients,
                frozenset({"192.168.1.10", "192.168.1.11"}),
            )

            env_path.write_text("BROKEN_LINE", encoding="utf-8")
            with self.assertRaisesRegex(ConfigError, "KEY=value"):
                load_controller_config({}, env_file=env_path)

    def test_allowlist_supports_ipv4_cidr_subnets_and_normalization(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = load_controller_config(
                {
                    "RASHKO_ENV_FILE": str(Path(directory) / "missing.env"),
                    "RASHKO_SHARED_SECRET": SECRET,
                    "RASHKO_ALLOWED_CLIENTS": (
                        "192.168.100.15/24, 169.254.0.0/16, 192.168.1.10, 10.0.0.5/32"
                    ),
                }
            )
            self.assertEqual(
                config.allowed_clients,
                frozenset(
                    {
                        "192.168.100.0/24",
                        "169.254.0.0/16",
                        "192.168.1.10",
                        "10.0.0.5/32",
                    }
                ),
            )
            self.assertTrue(is_client_allowed("192.168.100.42", config.allowed_clients))
            self.assertTrue(is_client_allowed("169.254.12.34", config.allowed_clients))
            self.assertTrue(is_client_allowed("192.168.1.10", config.allowed_clients))
            self.assertTrue(is_client_allowed("10.0.0.5", config.allowed_clients))
            self.assertFalse(is_client_allowed("192.168.101.42", config.allowed_clients))
            self.assertFalse(is_client_allowed("not-an-ip", config.allowed_clients))
            self.assertFalse(is_client_allowed("::1", config.allowed_clients))

    def test_allowlist_rejects_wildcards_ipv6_and_malformed_entries(self) -> None:
        invalid_cases = (
            ("0.0.0.0/0", r"/0 wildcard"),
            ("192.168.1.0/0", r"/0 wildcard"),
            ("::1", r"IPv4 only"),
            ("fe80::/64", r"IPv4 only"),
            ("::/0", r"IPv4 only"),
            ("not-an-ip", r"invalid RASHKO_ALLOWED_CLIENTS"),
            ("192.168.1.256", r"invalid RASHKO_ALLOWED_CLIENTS"),
            ("192.168.1.0/33", r"invalid RASHKO_ALLOWED_CLIENTS"),
            ("192.168.1.0/abc", r"invalid RASHKO_ALLOWED_CLIENTS"),
        )
        with tempfile.TemporaryDirectory() as directory:
            missing_env = str(Path(directory) / "missing.env")
            for raw_value, pattern in invalid_cases:
                with self.subTest(raw_value=raw_value):
                    with self.assertRaisesRegex(ConfigError, pattern):
                        load_controller_config(
                            {
                                "RASHKO_ENV_FILE": missing_env,
                                "RASHKO_SHARED_SECRET": SECRET,
                                "RASHKO_ALLOWED_CLIENTS": raw_value,
                            }
                        )

    def test_direct_controller_config_validates_allowed_clients(self) -> None:
        base_kwargs = dict(
            bind_host="127.0.0.1",
            port=5005,
            shared_secret=SECRET,
            allow_insecure=False,
            watchdog_ms=500,
            freshness_ms=5000,
            update_hz=50.0,
            max_sessions=8,
            hardware_backend="dry-run",
            servo_config_path=Path("unused.json"),
            hardware=default_hardware_config(),
        )
        valid = ControllerConfig(
            allowed_clients=frozenset({"192.168.100.0/24"}),
            **base_kwargs,
        )
        self.assertEqual(valid.allowed_clients, frozenset({"192.168.100.0/24"}))

        for bad_clients in (
            frozenset({"0.0.0.0/0"}),
            frozenset({"fe80::/64"}),
            frozenset({"::1"}),
            frozenset({"not-an-ip"}),
            frozenset({""}),
        ):
            with self.subTest(bad_clients=bad_clients):
                with self.assertRaises(ConfigError):
                    ControllerConfig(allowed_clients=bad_clients, **base_kwargs)


if __name__ == "__main__":
    unittest.main()
