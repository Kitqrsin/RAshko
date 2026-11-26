from __future__ import annotations

import errno
import json
import os
import socket
import sys
import tempfile
import threading
import unittest
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

from arm_controller import pi_endpoint

try:
    from rashko_communicator.config import ControllerConfig, default_hardware_config
    from rashko_communicator.controller import ArmController, UdpControllerServer
    from rashko_communicator.hardware import DryRunHardware

    _HAS_COMMUNICATOR = True
except ImportError:
    _HAS_COMMUNICATOR = False


_TEST_SECRET = "s" * 64


class _HealthHandler(BaseHTTPRequestHandler):
    status_code = HTTPStatus.OK
    payload: dict[str, object] = {"status": "ok", "backend": "synthetic"}

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.partition("?")[0]
        if path != "/healthz":
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")
            return
        body = json.dumps(self.payload).encode("utf-8")
        self.send_response(self.status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _format: str, *_args: object) -> None:
        return


def _start_health_server(
    payload: dict[str, object] | None = None,
    status_code: HTTPStatus = HTTPStatus.OK,
) -> tuple[ThreadingHTTPServer, int, threading.Thread]:
    class _ConfiguredHandler(_HealthHandler):
        pass

    _ConfiguredHandler.status_code = status_code
    _ConfiguredHandler.payload = (
        {"status": "ok", "backend": "synthetic"} if payload is None else payload
    )

    server = ThreadingHTTPServer(("127.0.0.1", 0), _ConfiguredHandler)
    port = int(server.server_address[1])
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, port, thread


def _start_udp_controller_server(
    *,
    secret: str = _TEST_SECRET,
    allow_insecure: bool = False,
) -> tuple[UdpControllerServer, ArmController, DryRunHardware, int, threading.Event, threading.Thread]:
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(("127.0.0.1", 0))
    port = int(udp_sock.getsockname()[1])
    hardware_cfg = default_hardware_config()
    config = ControllerConfig(
        bind_host="127.0.0.1",
        port=port,
        shared_secret=secret,
        allow_insecure=allow_insecure,
        allowed_clients=frozenset(),
        watchdog_ms=500,
        freshness_ms=5000,
        update_hz=50.0,
        max_sessions=8,
        hardware_backend="dry-run",
        servo_config_path=Path("unused.json"),
        hardware=hardware_cfg,
    )
    hardware = DryRunHardware(hardware_cfg)
    controller = ArmController(config, hardware)
    server = UdpControllerServer(config, controller, udp_socket=udp_sock)
    stop_event = threading.Event()
    thread = threading.Thread(
        target=server.serve_forever,
        args=(stop_event,),
        daemon=True,
    )
    thread.start()
    return server, controller, hardware, port, stop_event, thread


class ValidatePiHostTests(unittest.TestCase):
    def test_accepts_valid_ipv4_and_hostnames(self) -> None:
        self.assertEqual(pi_endpoint.validate_pi_host("192.168.1.50"), "192.168.1.50")
        self.assertEqual(pi_endpoint.validate_pi_host("  10.0.0.42 \t"), "10.0.0.42")
        self.assertEqual(pi_endpoint.validate_pi_host("rashko.local"), "rashko.local")
        self.assertEqual(pi_endpoint.validate_pi_host("pi-arm_01.lan"), "pi-arm_01.lan")

    def test_auto_handling(self) -> None:
        with self.assertRaises(ValueError):
            pi_endpoint.validate_pi_host("auto")
        with self.assertRaises(ValueError):
            pi_endpoint.validate_pi_host("AUTO")
        self.assertEqual(pi_endpoint.validate_pi_host(" AUTO ", allow_auto=True), "auto")

    def test_rejects_empty_and_injection_characters(self) -> None:
        invalid_hosts = [
            "",
            "   ",
            "192.168.1.50\nRASHKO_ALLOW_INSECURE=true",
            "192.168.1.50\rfoo",
            "192.168.1.50 extra",
            "192.168.1.50:5005",
            "http://192.168.1.50",
            "192.168.1.50/24",
            "host=1.2.3.4",
            "192.168.1.50\x00",
            ".leadingdot",
            "-leadingdash",
        ]
        for bad in invalid_hosts:
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    pi_endpoint.validate_pi_host(bad)


class DotenvAndEndpointResolutionTests(unittest.TestCase):
    def test_default_env_file_precedence(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            custom_env = Path(tmpdir) / ".env.custom"
            self.assertEqual(
                pi_endpoint.default_env_file({"RASHKO_ENV_FILE": str(custom_env)}),
                custom_env,
            )
            fake_container_env = Path(tmpdir) / ".env.windows"
            fake_container_env.write_text("RASHKO_PI_HOST=192.168.1.55\n", encoding="utf-8")
            with mock.patch.object(pi_endpoint, "DEFAULT_CONTAINER_ENV_FILE", fake_container_env):
                self.assertEqual(pi_endpoint.default_env_file({}), fake_container_env)
                self.assertEqual(
                    pi_endpoint.default_env_file({"RASHKO_ENV_FILE": ""}),
                    fake_container_env,
                )
            missing_container_env = Path(tmpdir) / "does-not-exist"
            with mock.patch.object(
                pi_endpoint, "DEFAULT_CONTAINER_ENV_FILE", missing_container_env
            ):
                self.assertIsNone(pi_endpoint.default_env_file({}))

    def test_load_dotenv_file_parses_valid_and_ignores_malformed(self) -> None:
        self.assertEqual(pi_endpoint.load_dotenv_file(None), {})
        with tempfile.TemporaryDirectory() as tmpdir:
            missing = Path(tmpdir) / "missing.env"
            self.assertEqual(pi_endpoint.load_dotenv_file(missing), {})

            env_path = Path(tmpdir) / ".env.windows"
            env_path.write_text(
                "\n"
                "# Comment line\n"
                "malformed_line_without_equals\n"
                "=missing_key\n"
                "BAD KEY=ignored\n"
                'export RASHKO_PI_HOST="192.168.55.10"\n'
                "RASHKO_SHARED_SECRET='secret#with=chars'\n"
                "RASHKO_CAMERA_URL=\n",
                encoding="utf-8",
            )
            parsed = pi_endpoint.load_dotenv_file(env_path)
            self.assertEqual(
                parsed,
                {
                    "RASHKO_PI_HOST": "192.168.55.10",
                    "RASHKO_SHARED_SECRET": "secret#with=chars",
                    "RASHKO_CAMERA_URL": "",
                },
            )

    def test_resolve_pi_endpoint_live_file_beats_stale_environ(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            env_path = Path(tmpdir) / ".env.windows"
            env_path.write_text(
                "RASHKO_PI_HOST=192.168.100.42\n"
                "RASHKO_PI_PORT=6006\n"
                "RASHKO_CAMERA_PORT=8090\n"
                "RASHKO_ETHERNET_SUBNET=192.168.100.0/24\n"
                "RASHKO_PI_HOSTNAME=rashko.local\n",
                encoding="utf-8",
            )
            stale_env = {
                "RASHKO_PI_HOST": "192.168.1.99",
                "RASHKO_PI_PORT": "5005",
                "RASHKO_CAMERA_PORT": "8080",
            }
            resolved = pi_endpoint.resolve_pi_endpoint(stale_env, env_file=env_path)
            self.assertEqual(
                resolved,
                {
                    "pi_host": "192.168.100.42",
                    "pi_port": 6006,
                    "camera_port": 8090,
                    "camera_url": "http://192.168.100.42:8090/stream.mjpg",
                    "subnet_hint": "192.168.100.0/24",
                    "pi_hostname": "rashko.local",
                },
            )

    def test_resolve_pi_endpoint_override_and_auto_fallback(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            env_path = Path(tmpdir) / ".env.windows"
            env_path.write_text("RASHKO_PI_HOST=auto\n", encoding="utf-8")

            with_override = pi_endpoint.resolve_pi_endpoint(
                {
                    "RASHKO_PI_HOST_OVERRIDE": "10.20.30.40",
                    "RASHKO_PI_HOST": "192.168.1.77",
                },
                env_file=env_path,
            )
            self.assertEqual(with_override["pi_host"], "10.20.30.40")
            self.assertEqual(
                with_override["camera_url"],
                "http://10.20.30.40:8080/stream.mjpg",
            )

            # "auto" in override and file falls back to env, then default_host.
            fallback_to_env = pi_endpoint.resolve_pi_endpoint(
                {"RASHKO_PI_HOST_OVERRIDE": "auto", "RASHKO_PI_HOST": "192.168.1.77"},
                env_file=env_path,
            )
            self.assertEqual(fallback_to_env["pi_host"], "192.168.1.77")

            fallback_to_default = pi_endpoint.resolve_pi_endpoint(
                {"RASHKO_PI_HOST": "auto"},
                env_file=env_path,
            )
            self.assertEqual(fallback_to_default["pi_host"], pi_endpoint.DEFAULT_PI_HOST)

    def test_resolve_pi_endpoint_invalid_ports_raise(self) -> None:
        with self.assertRaises(ValueError):
            pi_endpoint.resolve_pi_endpoint({"RASHKO_PI_PORT": "70000"}, env_file="")
        with self.assertRaises(ValueError):
            pi_endpoint.resolve_pi_endpoint({"RASHKO_CAMERA_PORT": "abc"}, env_file="")

    def test_default_camera_url_empty_when_unconfigured(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            env_path = Path(tmpdir) / ".env.windows"
            env_path.write_text("RASHKO_PI_HOST=\nRASHKO_CAMERA_URL=\n", encoding="utf-8")

            self.assertEqual(pi_endpoint.default_camera_url({}, env_file=env_path), "")
            self.assertEqual(
                pi_endpoint.default_camera_url(
                    {"RASHKO_PI_HOST": "", "RASHKO_CAMERA_URL": ""},
                    env_file=Path(tmpdir) / "missing.env",
                ),
                "",
            )

            env_path.write_text("RASHKO_PI_HOST=192.168.10.20\n", encoding="utf-8")
            self.assertEqual(
                pi_endpoint.default_camera_url({}, env_file=env_path),
                "http://192.168.10.20:8080/stream.mjpg",
            )

            env_path.write_text(
                "RASHKO_PI_HOST=192.168.10.20\nRASHKO_CAMERA_URL=http://cam.local:9000/mjpg\n",
                encoding="utf-8",
            )
            self.assertEqual(
                pi_endpoint.default_camera_url({}, env_file=env_path),
                "http://cam.local:9000/mjpg",
            )


class UpdateDotenvTests(unittest.TestCase):
    def test_update_dotenv_preserves_crlf_comments_and_secret_bytes(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            env_path = Path(tmpdir) / ".env.windows"
            secret_line = b"RASHKO_SHARED_SECRET=Keep$Exact!Secret==With#Symbols_1234567890"
            original = (
                b"# Header comment\r\n"
                b"RASHKO_PI_HOST=192.168.1.50\r\n"
                + secret_line
                + b"\r\n"
                b"# Trailing comment\r\n"
                b"RASHKO_CAMERA_PORT=8080\r\n"
            )
            env_path.write_bytes(original)

            returned = pi_endpoint.update_dotenv_pi_host("192.168.100.88", env_file=env_path)
            self.assertEqual(returned, env_path)

            updated = env_path.read_bytes()
            expected = original.replace(
                b"RASHKO_PI_HOST=192.168.1.50",
                b"RASHKO_PI_HOST=192.168.100.88",
            )
            self.assertEqual(updated, expected)
            self.assertIn(secret_line + b"\r\n", updated)

    def test_update_dotenv_appends_when_absent_and_creates_when_missing(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            env_path = Path(tmpdir) / ".env.windows"
            env_path.write_bytes(b"RASHKO_SHARED_SECRET=abc\r\nRASHKO_PI_PORT=5005")
            pi_endpoint.update_dotenv_pi_host("auto", env_file=env_path)
            self.assertEqual(
                env_path.read_bytes(),
                b"RASHKO_SHARED_SECRET=abc\r\nRASHKO_PI_PORT=5005\r\nRASHKO_PI_HOST=auto\r\n",
            )

            new_path = Path(tmpdir) / "nested" / ".env.new"
            pi_endpoint.update_dotenv_pi_host("10.0.0.5", env_file=new_path)
            self.assertEqual(new_path.read_bytes(), b"RASHKO_PI_HOST=10.0.0.5\n")

    def test_update_dotenv_keeps_original_if_replace_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            env_path = Path(tmpdir) / ".env.windows"
            env_path.write_bytes(b"RASHKO_PI_HOST=192.168.1.50\nRASHKO_SHARED_SECRET=keep\n")

            with mock.patch(
                "os.replace",
                side_effect=OSError(errno.EBUSY, "Device or resource busy"),
            ):
                with self.assertRaises(OSError):
                    pi_endpoint.update_dotenv_pi_host("192.168.1.75", env_file=env_path)

            self.assertEqual(
                env_path.read_bytes(),
                b"RASHKO_PI_HOST=192.168.1.50\nRASHKO_SHARED_SECRET=keep\n",
            )
            # Temp file should be cleaned up.
            self.assertEqual([p.name for p in Path(tmpdir).iterdir()], [".env.windows"])


class ProbeAndDiscoveryTests(unittest.TestCase):
    def test_probe_camera_health_only_and_without_communicator(self) -> None:
        server, cam_port, thread = _start_health_server()
        try:
            with mock.patch.dict(sys.modules, {"rashko_communicator": None}):
                result = pi_endpoint.probe_pi_candidate(
                    "127.0.0.1",
                    camera_port=cam_port,
                    secret=_TEST_SECRET,
                    timeout_s=1.0,
                )
            self.assertTrue(result["reachable"])
            self.assertTrue(result["camera_ok"])
            self.assertFalse(result["udp_ok"])
            self.assertEqual(result["host"], "127.0.0.1")
            self.assertEqual(result["state"], [])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

    def test_probe_rejects_unhealthy_or_non_rashko_http(self) -> None:
        server, cam_port, thread = _start_health_server(
            payload={"status": "ok", "backend": "other-app"}
        )
        try:
            result = pi_endpoint.probe_pi_candidate(
                "127.0.0.1",
                camera_port=cam_port,
                timeout_s=0.5,
            )
            self.assertFalse(result["reachable"])
            self.assertFalse(result["camera_ok"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

    @unittest.skipUnless(_HAS_COMMUNICATOR, "rashko_communicator is not on PYTHONPATH")
    def test_probe_pi_candidate_real_http_and_udp_remains_safe_readonly(self) -> None:
        http_server, cam_port, http_thread = _start_health_server(
            payload={"status": "ok", "backend": "picamera2"}
        )
        udp_server, controller, hardware, udp_port, stop_event, udp_thread = (
            _start_udp_controller_server(secret=_TEST_SECRET)
        )
        try:
            result = pi_endpoint.probe_pi_candidate(
                "127.0.0.1",
                pi_port=udp_port,
                camera_port=cam_port,
                secret=_TEST_SECRET,
                timeout_s=1.0,
            )
            self.assertTrue(result["reachable"])
            self.assertTrue(result["camera_ok"])
            self.assertTrue(result["udp_ok"])
            self.assertIn("ESTOP_LATCHED", result["state"])
            self.assertNotIn("OUTPUTS_ENABLED", result["state"])

            # Verify read-only safety: E-stop stays latched, outputs disabled, no writes.
            self.assertTrue(controller.estop_latched)
            self.assertFalse(hardware.outputs_enabled)
            self.assertEqual(hardware.write_count, 0)
        finally:
            stop_event.set()
            udp_thread.join(timeout=2.0)
            udp_server.close()
            http_server.shutdown()
            http_server.server_close()
            http_thread.join(timeout=2.0)

    @unittest.skipUnless(_HAS_COMMUNICATOR, "rashko_communicator is not on PYTHONPATH")
    def test_discover_pi_host_via_subnet_scan_and_hostname_and_unverified_returns_none(self) -> None:
        http_server, cam_port, http_thread = _start_health_server()
        udp_server, controller, hardware, udp_port, stop_event, udp_thread = (
            _start_udp_controller_server(secret=_TEST_SECRET)
        )
        try:
            # 1) Discover via subnet_hint scan (prefix < 24 capped; /30 keeps test fast).
            found = pi_endpoint.discover_pi_host(
                current_host="auto",
                subnet_hint="127.0.0.0/30",
                extra_hostnames=(),
                pi_port=udp_port,
                camera_port=cam_port,
                secret=_TEST_SECRET,
                timeout_s=0.4,
            )
            self.assertIsNotNone(found)
            assert found is not None
            self.assertEqual(found["host"], "127.0.0.1")
            self.assertTrue(found["udp_ok"])
            self.assertTrue(found["camera_ok"])

            # 2) Discover via extra_hostnames ("localhost" -> 127.0.0.1 via getaddrinfo).
            with mock.patch.object(
                pi_endpoint, "DEFAULT_MDNS_HOSTNAMES", ()
            ):
                found_name = pi_endpoint.discover_pi_host(
                    extra_hostnames=("localhost",),
                    pi_port=udp_port,
                    camera_port=cam_port,
                    secret=_TEST_SECRET,
                    timeout_s=0.4,
                )
            self.assertIsNotNone(found_name)
            assert found_name is not None
            self.assertEqual(found_name["host"], "127.0.0.1")

            # 3) Never return an unverified host when nothing responds.
            unused_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            unused_sock.bind(("127.0.0.1", 0))
            dead_port = int(unused_sock.getsockname()[1])
            unused_sock.close()
            with mock.patch.object(
                pi_endpoint, "DEFAULT_MDNS_HOSTNAMES", ()
            ):
                missing = pi_endpoint.discover_pi_host(
                    current_host="127.0.0.1",
                    subnet_hint="127.0.0.0/30",
                    pi_port=dead_port,
                    camera_port=dead_port,
                    secret=_TEST_SECRET,
                    timeout_s=0.15,
                )
            self.assertIsNone(missing)

            # Controller safety still intact after discovery probes.
            self.assertTrue(controller.estop_latched)
            self.assertFalse(hardware.outputs_enabled)
            self.assertEqual(hardware.write_count, 0)
        finally:
            stop_event.set()
            udp_thread.join(timeout=2.0)
            udp_server.close()
            http_server.shutdown()
            http_server.server_close()
            http_thread.join(timeout=2.0)

    def test_camera_only_candidate_cannot_be_selected(self) -> None:
        with (
            mock.patch.object(pi_endpoint, "DEFAULT_MDNS_HOSTNAMES", ()),
            mock.patch.object(pi_endpoint, "probe_pi_candidate",
                              return_value={"reachable": True, "udp_ok": False}),
        ):
            self.assertIsNone(pi_endpoint.discover_pi_host(
                current_host="127.0.0.1", secret=_TEST_SECRET,
            ))

    def test_discover_prefers_udp_ok_over_camera_only_and_caps_large_prefix(self) -> None:
        net = pi_endpoint._network_from_spec("192.168.50.12/16")
        self.assertEqual(str(net), "192.168.50.0/24")

        def fake_probe(
            host: str,
            *,
            pi_port: int = 5005,
            camera_port: int = 8080,
            secret: str = "",
            allow_insecure: bool = False,
            timeout_s: float = 0.6,
        ) -> dict[str, object]:
            if host == "192.168.50.10":
                return {
                    "host": host,
                    "reachable": True,
                    "udp_ok": False,
                    "camera_ok": True,
                    "state": [],
                    "detail": "camera only",
                }
            if host == "192.168.50.20":
                return {
                    "host": host,
                    "reachable": True,
                    "udp_ok": True,
                    "camera_ok": False,
                    "state": ["ESTOP_LATCHED"],
                    "detail": "udp only",
                }
            return {
                "host": host,
                "reachable": False,
                "udp_ok": False,
                "camera_ok": False,
                "state": [],
                "detail": "unreachable",
            }

        with (
            mock.patch.object(pi_endpoint, "DEFAULT_MDNS_HOSTNAMES", ()),
            mock.patch.object(pi_endpoint, "probe_pi_candidate", side_effect=fake_probe),
        ):
            best = pi_endpoint.discover_pi_host(
                current_host="192.168.50.10",
                subnet_hint="192.168.50.0/24",
                secret=_TEST_SECRET,
                timeout_s=0.1,
            )
        self.assertIsNotNone(best)
        assert best is not None
        self.assertEqual(best["host"], "192.168.50.20")
        self.assertTrue(best["udp_ok"])


if __name__ == "__main__":
    unittest.main()
