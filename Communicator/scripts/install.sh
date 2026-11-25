#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run this installer with sudo: sudo bash scripts/install.sh" >&2
    exit 1
fi

SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="/opt/rashko-communicator"
VENV_DIR="${INSTALL_DIR}/.venv"
SERVICE_USER="rashko"
ENV_FILE="/etc/rashko-controller.env"
SERVO_FILE="/etc/rashko-servos.json"

if ! command -v apt-get >/dev/null 2>&1; then
    echo "This installer supports Raspberry Pi OS/Debian (apt-get is required)." >&2
    exit 1
fi
if ! command -v systemctl >/dev/null 2>&1; then
    echo "systemd is required." >&2
    exit 1
fi

apt-get update
apt-get install -y --no-install-recommends \
    i2c-tools \
    python3-gpiozero \
    python3-lgpio \
    python3-picamera2 \
    python3-pip \
    python3-venv \
    rpicam-apps

if ! id -u "${SERVICE_USER}" >/dev/null 2>&1; then
    useradd \
        --system \
        --create-home \
        --home-dir /var/lib/rashko \
        --shell /usr/sbin/nologin \
        "${SERVICE_USER}"
fi

for group_name in i2c gpio video render; do
    if getent group "${group_name}" >/dev/null 2>&1; then
        usermod -a -G "${group_name}" "${SERVICE_USER}"
    fi
done

install -d -o root -g root -m 0755 "${INSTALL_DIR}"
install -d -o "${SERVICE_USER}" -g "${SERVICE_USER}" -m 0750 /var/lib/rashko

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
    python3 -m venv --system-site-packages "${VENV_DIR}"
fi
"${VENV_DIR}/bin/python" -m pip install --upgrade pip setuptools wheel
"${VENV_DIR}/bin/python" -m pip install --upgrade "${SOURCE_DIR}[hardware]"

if [[ ! -e "${ENV_FILE}" ]]; then
    install \
        -o root -g "${SERVICE_USER}" -m 0640 \
        "${SOURCE_DIR}/config/rashko-controller.env.example" \
        "${ENV_FILE}"
    echo "Created ${ENV_FILE}; it still requires the laptop secret and IP address."
else
    echo "Preserved existing ${ENV_FILE}."
fi

if [[ ! -e "${SERVO_FILE}" ]]; then
    install \
        -o root -g "${SERVICE_USER}" -m 0640 \
        "${SOURCE_DIR}/config/rashko-servos.json" \
        "${SERVO_FILE}"
    echo "Installed unverified migration calibration at ${SERVO_FILE}."
else
    echo "Preserved existing ${SERVO_FILE}."
fi

install -o root -g root -m 0644 \
    "${SOURCE_DIR}/systemd/rashko-controller.service" \
    /etc/systemd/system/rashko-controller.service
install -o root -g root -m 0644 \
    "${SOURCE_DIR}/systemd/rashko-camera.service" \
    /etc/systemd/system/rashko-camera.service
systemctl daemon-reload

echo
echo "Installation complete. Services were not started."
echo "1. Edit ${ENV_FILE} and keep RASHKO_HARDWARE_BACKEND=dry-run."
echo "2. Run the tests and dry-run commissioning steps in README.md."
echo "3. Start with: systemctl enable --now rashko-controller rashko-camera"
