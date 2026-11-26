"""Endpoint resolution, dotenv updates, and safe Pi discovery helpers."""

from __future__ import annotations

import concurrent.futures
import ipaddress
import json
import os
import re
import socket
import tempfile
import urllib.error
import urllib.request
from collections.abc import Mapping, Sequence
from pathlib import Path

DEFAULT_CONTAINER_ENV_FILE = Path("/workspace/.env.windows")
DEFAULT_PI_HOST = "192.168.1.50"
DEFAULT_PI_PORT = 5005
DEFAULT_CAMERA_PORT = 8080
DEFAULT_MDNS_HOSTNAMES = ("rashko.local", "raspberrypi.local", "puppypi.local")

_ENV_KEY_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_PI_HOST_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")
_PI_HOST_LINE_RE = re.compile(r"^(\s*(?:export\s+)?)RASHKO_PI_HOST\s*=.*$")
_VALID_CAMERA_BACKENDS = frozenset({"picamera2", "synthetic"})


def default_env_file(environ: Mapping[str, str] | None = None) -> Path | None:
    """Return the active dotenv path from ``RASHKO_ENV_FILE`` or container mount."""
    env = os.environ if environ is None else environ
    configured = str(env.get("RASHKO_ENV_FILE", "")).strip()
    if configured:
        return Path(configured)
    if DEFAULT_CONTAINER_ENV_FILE.is_file():
        return DEFAULT_CONTAINER_ENV_FILE
    return None


def load_dotenv_file(path: Path | str | None) -> dict[str, str]:
    """Parse ``KEY=VALUE`` pairs from a dotenv file without raising on malformed lines."""
    if path is None:
        return {}
    if isinstance(path, str) and not path.strip():
        return {}
    file_path = Path(path)
    try:
        if not file_path.is_file():
            return {}
        text = file_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return {}

    values: dict[str, str] = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export ") or line.startswith("export\t"):
            line = line[6:].lstrip()
        if "=" not in line:
            continue
        key, raw_value = line.split("=", 1)
        key = key.strip()
        if not _ENV_KEY_RE.fullmatch(key):
            continue
        value = raw_value.strip()
        if len(value) >= 2 and (
            (value[0] == '"' and value[-1] == '"')
            or (value[0] == "'" and value[-1] == "'")
        ):
            value = value[1:-1]
        values[key] = value
    return values


def validate_pi_host(host: str, *, allow_auto: bool = False) -> str:
    """Validate an IPv4 literal or DNS/mDNS hostname (and optionally ``auto``)."""
    if not isinstance(host, str):
        raise ValueError("pi_host must be a string")
    cleaned = host.strip()
    if not cleaned:
        raise ValueError("pi_host must not be empty")
    if cleaned.lower() == "auto":
        if allow_auto:
            return "auto"
        raise ValueError("pi_host 'auto' must be resolved to an IPv4 address or hostname")
    if not _PI_HOST_RE.fullmatch(cleaned):
        raise ValueError(f"invalid pi_host: {host!r}")
    return cleaned


def _resolve_port(
    file_values: Mapping[str, str],
    env: Mapping[str, str],
    key: str,
    default: int,
) -> int:
    raw = str(file_values.get(key, "")).strip() or str(env.get(key, "")).strip()
    if not raw:
        return default
    try:
        port = int(raw)
    except ValueError as exc:
        raise ValueError(f"{key} must be an integer in 1..65535, got {raw!r}") from exc
    if not 1 <= port <= 65535:
        raise ValueError(f"{key} must be in 1..65535, got {port}")
    return port


def resolve_pi_endpoint(
    environ: Mapping[str, str] | None = None,
    *,
    env_file: Path | str | None = None,
    default_host: str = DEFAULT_PI_HOST,
    file_values: Mapping[str, str] | None = None,
) -> dict[str, object]:
    """Resolve Pi UDP and camera endpoints with live dotenv precedence."""
    env = os.environ if environ is None else environ
    resolved_env_file = env_file if env_file is not None else default_env_file(env)
    if file_values is None:
        file_values = load_dotenv_file(resolved_env_file)

    override = str(env.get("RASHKO_PI_HOST_OVERRIDE", "")).strip()
    file_host = str(file_values.get("RASHKO_PI_HOST", "")).strip()
    env_host = str(env.get("RASHKO_PI_HOST", "")).strip()

    if override and override.lower() != "auto":
        raw_host = override
    elif file_host and file_host.lower() != "auto":
        raw_host = file_host
    elif env_host and env_host.lower() != "auto":
        raw_host = env_host
    else:
        raw_host = default_host

    pi_host = validate_pi_host(raw_host)
    pi_port = _resolve_port(file_values, env, "RASHKO_PI_PORT", DEFAULT_PI_PORT)
    camera_port = _resolve_port(file_values, env, "RASHKO_CAMERA_PORT", DEFAULT_CAMERA_PORT)

    explicit_camera_url = (
        str(file_values.get("RASHKO_CAMERA_URL", "")).strip()
        or str(env.get("RASHKO_CAMERA_URL", "")).strip()
    )
    if explicit_camera_url:
        camera_url = explicit_camera_url
    elif pi_host:
        camera_url = f"http://{pi_host}:{camera_port}/stream.mjpg"
    else:
        camera_url = ""

    subnet_hint = (
        str(file_values.get("RASHKO_ETHERNET_SUBNET", "")).strip()
        or str(env.get("RASHKO_ETHERNET_SUBNET", "")).strip()
    )
    pi_hostname = (
        str(file_values.get("RASHKO_PI_HOSTNAME", "")).strip()
        or str(env.get("RASHKO_PI_HOSTNAME", "")).strip()
    )

    return {
        "pi_host": pi_host,
        "pi_port": pi_port,
        "camera_port": camera_port,
        "camera_url": camera_url,
        "subnet_hint": subnet_hint,
        "pi_hostname": pi_hostname,
    }


def default_camera_url(
    environ: Mapping[str, str] | None = None,
    *,
    env_file: Path | str | None = None,
) -> str:
    """Return the resolved camera URL, or ``""`` if no Pi host or camera URL is configured."""
    env = os.environ if environ is None else environ
    resolved_env_file = env_file if env_file is not None else default_env_file(env)
    file_values = load_dotenv_file(resolved_env_file)

    override = str(env.get("RASHKO_PI_HOST_OVERRIDE", "")).strip()
    has_override = bool(override and override.lower() != "auto")
    has_configured = has_override or any(
        bool(str(source.get(key, "")).strip())
        for source in (file_values, env)
        for key in ("RASHKO_CAMERA_URL", "RASHKO_PI_HOST")
    )
    if not has_configured:
        return ""
    return str(resolve_pi_endpoint(env, env_file=resolved_env_file, file_values=file_values)["camera_url"])


def _split_line_ending(line: str) -> tuple[str, str]:
    if line.endswith("\r\n"):
        return line[:-2], "\r\n"
    if line.endswith("\n"):
        return line[:-1], "\n"
    if line.endswith("\r"):
        return line[:-1], "\r"
    return line, ""


def update_dotenv_pi_host(host: str, env_file: Path | str | None = None) -> Path:
    """Update or append ``RASHKO_PI_HOST`` in a dotenv file while preserving other content."""
    cleaned_host = validate_pi_host(host, allow_auto=True)
    target = (
        Path(env_file)
        if env_file is not None
        else (default_env_file() or DEFAULT_CONTAINER_ENV_FILE)
    )

    if target.exists():
        raw_text = target.read_bytes().decode("utf-8")
        newline = "\r\n" if "\r\n" in raw_text else "\n"
        lines = raw_text.splitlines(keepends=True)
        new_lines: list[str] = []
        replaced = False
        for line in lines:
            body, ending = _split_line_ending(line)
            match = _PI_HOST_LINE_RE.match(body)
            if match:
                prefix = match.group(1)
                new_lines.append(f"{prefix}RASHKO_PI_HOST={cleaned_host}{ending}")
                replaced = True
            else:
                new_lines.append(line)
        if not replaced:
            if new_lines:
                last_body, last_ending = _split_line_ending(new_lines[-1])
                if not last_ending and last_body:
                    new_lines[-1] = f"{last_body}{newline}"
            new_lines.append(f"RASHKO_PI_HOST={cleaned_host}{newline}")
        new_text = "".join(new_lines)
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        new_text = f"RASHKO_PI_HOST={cleaned_host}\n"

    payload = new_text.encode("utf-8")
    tmp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=target.parent,
            prefix=f".{target.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            tmp_path = Path(handle.name)
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_path, target)
    except BaseException:
        if tmp_path is not None:
            tmp_path.unlink(missing_ok=True)
        raise

    return target


def _probe_camera_health(host: str, camera_port: int, timeout_s: float) -> tuple[bool, str]:
    url = f"http://{host}:{camera_port}/healthz"
    request = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            status = getattr(response, "status", response.getcode())
            if status != 200:
                return False, f"camera HTTP {status}"
            body = response.read(4096)
    except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
        return False, f"camera unreachable ({exc})"

    try:
        data = json.loads(body.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        return False, f"camera invalid JSON ({exc})"

    if not isinstance(data, dict):
        return False, "camera response is not a JSON object"
    backend = data.get("backend")
    if data.get("status") == "ok" and backend in _VALID_CAMERA_BACKENDS:
        return True, f"camera ok ({backend})"
    return False, f"camera unhealthy (status={data.get('status')!r}, backend={backend!r})"


def _probe_udp_status(
    host: str,
    pi_port: int,
    *,
    secret: str,
    allow_insecure: bool,
    timeout_s: float,
) -> tuple[bool, list[str], str]:
    try:
        from rashko_communicator import AckCode, ControllerState, UdpArmClient
    except ImportError:
        return False, [], "rashko_communicator unavailable"

    try:
        with UdpArmClient(
            host,
            pi_port,
            secret=secret,
            allow_insecure=allow_insecure,
        ) as client:
            sequence = client.send_status()
            ack = client.wait_for_ack(sequence, timeout_s=timeout_s)
    except Exception as exc:
        return False, [], f"udp unreachable ({exc})"

    state_list = [
        flag.name
        for flag in ControllerState
        if flag.value and (flag in ack.state) and flag.name
    ]
    if ack.code is AckCode.OK:
        return True, state_list, f"udp ok ({ack.code.name})"
    return False, state_list, f"udp ack ({ack.code.name})"


def probe_pi_candidate(
    host: str,
    *,
    pi_port: int = DEFAULT_PI_PORT,
    camera_port: int = DEFAULT_CAMERA_PORT,
    secret: str = "",
    allow_insecure: bool = False,
    timeout_s: float = 0.6,
) -> dict[str, object]:
    """Perform a read-only camera /healthz and optional UDP status probe."""
    cleaned_host = validate_pi_host(host)
    candidates = _resolve_ipv4_candidates(cleaned_host)
    if candidates:
        cleaned_host = candidates[0]
    udp_ok = False
    state_list: list[str] = []
    details: list[str] = []
    if secret or allow_insecure:
        udp_ok, state_list, udp_detail = _probe_udp_status(
            cleaned_host,
            pi_port,
            secret=secret,
            allow_insecure=allow_insecure,
            timeout_s=timeout_s,
        )
        details.append(udp_detail)
    camera_ok, camera_detail = _probe_camera_health(cleaned_host, camera_port, timeout_s)
    details.append(camera_detail)

    return {
        "host": cleaned_host,
        "reachable": bool(udp_ok or camera_ok),
        "udp_ok": udp_ok,
        "camera_ok": camera_ok,
        "state": state_list,
        "detail": "; ".join(details),
    }


def _resolve_ipv4_candidates(host: str) -> list[str]:
    try:
        addr = ipaddress.ip_address(host)
    except ValueError:
        pass
    else:
        return [str(addr)] if addr.version == 4 else []

    try:
        infos = socket.getaddrinfo(host, None, family=socket.AF_INET)
    except OSError:
        return []

    resolved: list[str] = []
    for info in infos:
        sockaddr = info[4]
        if not sockaddr:
            continue
        ip = str(sockaddr[0])
        if ip not in resolved:
            resolved.append(ip)
    return resolved


def _network_from_spec(spec: str, *, require_private: bool = False) -> ipaddress.IPv4Network | None:
    cleaned = spec.strip()
    if not cleaned or cleaned.lower() == "auto":
        return None
    try:
        if "/" in cleaned:
            iface = ipaddress.ip_interface(cleaned)
            if iface.version != 4:
                return None
            if require_private and (not iface.ip.is_private or iface.ip.is_loopback):
                return None
            if iface.network.prefixlen < 24:
                return ipaddress.IPv4Network(f"{iface.ip}/24", strict=False)
            return iface.network
        addr = ipaddress.ip_address(cleaned)
    except ValueError:
        return None
    if addr.version != 4:
        return None
    if require_private and (not addr.is_private or addr.is_loopback):
        return None
    return ipaddress.IPv4Network(f"{addr}/24", strict=False)


def discover_pi_host(
    *,
    current_host: str = "",
    subnet_hint: str = "",
    extra_hostnames: Sequence[str] = (),
    pi_port: int = DEFAULT_PI_PORT,
    camera_port: int = DEFAULT_CAMERA_PORT,
    secret: str = "",
    allow_insecure: bool = False,
    timeout_s: float = 0.35,
) -> dict[str, object] | None:
    """Locate a verified RAshko Pi endpoint via direct/mDNS names then /24 scan."""
    if not secret:
        return None
    seen_ips: set[str] = set()

    direct_candidates: list[str] = []
    for raw in (current_host, *extra_hostnames, *DEFAULT_MDNS_HOSTNAMES):
        if not isinstance(raw, str):
            continue
        try:
            cleaned = validate_pi_host(raw)
        except ValueError:
            continue
        if cleaned not in direct_candidates:
            direct_candidates.append(cleaned)

    for candidate in direct_candidates:
        for ip in _resolve_ipv4_candidates(candidate):
            if ip in seen_ips:
                continue
            seen_ips.add(ip)
            result = probe_pi_candidate(
                ip,
                pi_port=pi_port,
                camera_port=camera_port,
                secret=secret,
                allow_insecure=allow_insecure,
                timeout_s=timeout_s,
            )
            if result["udp_ok"]:
                return result

    networks: list[ipaddress.IPv4Network] = []
    hint_net = _network_from_spec(subnet_hint, require_private=False)
    if hint_net is not None:
        networks.append(hint_net)

    current_net = _network_from_spec(current_host, require_private=True)
    if current_net is not None and not networks:
        networks.append(current_net)

    for network in networks:
        hosts_to_scan = [str(ip) for ip in network.hosts() if str(ip) not in seen_ips]
        if not hosts_to_scan:
            continue
        seen_ips.update(hosts_to_scan)
        pool = concurrent.futures.ThreadPoolExecutor(max_workers=64)
        try:
            futures = [
                pool.submit(
                    probe_pi_candidate,
                    ip,
                    pi_port=pi_port,
                    camera_port=camera_port,
                    secret=secret,
                    allow_insecure=allow_insecure,
                    timeout_s=timeout_s,
                )
                for ip in hosts_to_scan
            ]
            for future in concurrent.futures.as_completed(futures):
                try:
                    res = future.result()
                except Exception:
                    continue
                if res.get("udp_ok"):
                    return res
        finally:
            pool.shutdown(wait=True, cancel_futures=True)
    return None
