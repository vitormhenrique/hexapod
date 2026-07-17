#!/usr/bin/env zsh

set -euo pipefail

workspace_dir="${0:A:h:h}"
monorepo_root="${workspace_dir:h}"
ros_env="${1:-jazzy}"
host="${2:-127.0.0.1}"
port="${3:-5560}"
token="${4:-hexapod-sim}"
rviz="${5:-true}"

if [[ "$host" != "127.0.0.1" && "$host" != "localhost" && "$host" != "::1" ]]; then
  print -u2 "companion simulation only permits a loopback host"
  exit 2
fi

if [[ "$rviz" != "true" && "$rviz" != "false" ]]; then
  print -u2 "companion simulation rviz must be true or false"
  exit 2
fi

endpoint="tcp://${host}:${port}?token=${token}"

(
  cd "$workspace_dir"
  export PYTHONPATH="${monorepo_root}/protocol/python${PYTHONPATH:+:${PYTHONPATH}}"
  exec pixi run -e "$ros_env" --manifest-path "$workspace_dir/pixi.toml" \
    "$workspace_dir/scripts/with_overlay.sh" \
    ros2 launch hexapod_controller_ros companion_sim.launch.py \
    "host:=${host}" "port:=${port}" "token:=${token}" "rviz:=${rviz}"
) &
ros_pid=$!

cleanup() {
  trap - EXIT INT TERM
  if kill -0 "$ros_pid" 2>/dev/null; then
    kill -INT "$ros_pid" 2>/dev/null || true
    wait "$ros_pid" || true
  fi
}

trap cleanup EXIT INT TERM

wait_for_endpoint() {
  print "Waiting for simulated firmware at ${endpoint}..."
  uv run python - "$host" "$port" "$token" "$ros_pid" <<'PY'
import os
import socket
import sys
import time

host, port_text, token, ros_pid_text = sys.argv[1:]
port = int(port_text)
ros_pid = int(ros_pid_text)
deadline = time.monotonic() + 15.0

while time.monotonic() < deadline:
    try:
        os.kill(ros_pid, 0)
    except OSError:
        print("ROS simulation exited before the companion endpoint became ready", file=sys.stderr)
        sys.exit(1)

    try:
        with socket.create_connection((host, port), timeout=0.25) as connection:
            connection.settimeout(0.25)
            connection.sendall(f"HEXAPOD_RELAY/1 {token}\n".encode("ascii"))
            connection.shutdown(socket.SHUT_WR)
            if connection.recv(32) == b"OK\n":
                sys.exit(0)
    except OSError:
        pass
    time.sleep(0.1)

print(f"Timed out waiting for simulated firmware at {host}:{port}", file=sys.stderr)
sys.exit(1)
PY
}

cd "$monorepo_root/companion"
if ! wait_for_endpoint; then
  exit 1
fi

print "Starting companion connected to ${endpoint}"
HEXAPOD_ENDPOINT="$endpoint" uv run hexapod-companion --endpoint "$endpoint"