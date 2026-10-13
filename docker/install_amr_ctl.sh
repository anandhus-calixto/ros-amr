#!/usr/bin/env bash
# Installs the restart helper (amr-ctl) as a systemd service on the robot HOST.
# Run once, as root, on the i.MX8M Plus (not inside the container):
#     bash /root/ros2_ws/src/ros-amr/docker/install_amr_ctl.sh
# Safe to run again (e.g. after editing amr-ctl.service or amr_ctl.py).
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run as root" >&2; exit 1; }
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$DIR/amr-ctl.service" ] || { echo "missing $DIR/amr-ctl.service" >&2; exit 1; }
python3 -c "import ast,sys; ast.parse(open('$DIR/../web_hmi/amr_ctl.py').read())"
install -m 644 "$DIR/amr-ctl.service" /etc/systemd/system/amr-ctl.service
systemctl daemon-reload
systemctl enable amr-ctl.service
systemctl restart amr-ctl.service
sleep 1
systemctl --no-pager --lines=3 status amr-ctl.service || true
echo
echo "amr-ctl status:"; curl -s --max-time 5 http://127.0.0.1:8081/ctl/status || echo "(no answer yet)"
echo
