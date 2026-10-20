#!/usr/bin/env bash
# Install the device bridge as a systemd service.
#
#   sudo deploy/install.sh config/turtlebot3_d435.yaml [PREFIX]
#
# PREFIX defaults to /opt/sodero-device-bridge. The script copies the program
# and the configuration there, creates a Python virtual environment and starts
# the service. When the system Python cannot load pyrealsense2 (Debian 12: no
# wheel for its Python 3.11), it uses uv to get a standalone Python 3.10, whose
# aarch64 wheels are built for an older glibc.
set -euo pipefail

config=${1:?usage: sudo deploy/install.sh CONFIG.yaml [PREFIX]}
prefix=${2:-/opt/sodero-device-bridge}
source_dir=$(cd "$(dirname "$0")/.." && pwd)
service_user=${SUDO_USER:-$(id -un)}

[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }
[ -f "$config" ] || { echo "no such configuration: $config" >&2; exit 1; }

echo "installing into $prefix (service user: $service_user)"
install -d "$prefix"
rm -rf "$prefix/device_bridge"
cp -r "$source_dir/device_bridge" "$prefix/"
install -m 644 "$source_dir/requirements.txt" "$prefix/requirements.txt"
install -m 644 "$config" "$prefix/config.yaml"

install_requirements() {
    "$prefix/venv/bin/python" -m pip install --quiet --upgrade pip &&
    "$prefix/venv/bin/python" -m pip install --quiet -r "$prefix/requirements.txt" &&
    "$prefix/venv/bin/python" -c "import pyrealsense2" 2>/dev/null
}

if python3 -m venv --clear "$prefix/venv" && install_requirements; then
    echo "using the system Python ($(python3 --version))"
else
    echo "the system Python cannot load pyrealsense2: using a standalone Python 3.10"
    export UV_INSTALL_DIR=/usr/local/bin UV_NO_MODIFY_PATH=1 UV_PYTHON_INSTALL_DIR="$prefix/python"
    command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
    uv venv --clear --seed --python 3.10 "$prefix/venv"
    install_requirements || { echo "could not install the requirements" >&2; exit 1; }
fi
chown -R "$service_user": "$prefix"

sed -e "s#@PREFIX@#$prefix#g" -e "s#@USER@#$service_user#g" \
    "$source_dir/deploy/sodero-device-bridge.service" > /etc/systemd/system/sodero-device-bridge.service
systemctl daemon-reload
systemctl enable sodero-device-bridge.service
systemctl restart sodero-device-bridge.service
echo "installed; follow the log with: journalctl -u sodero-device-bridge -f"
