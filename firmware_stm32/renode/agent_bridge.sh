#!/usr/bin/env bash
# Ф3 micro-ROS agent round-trip, fully in emulation (no board, no real serial port):
#
#   firmware USART2  <--Renode socket (raw, telnet OFF)-->  socat  <--pty-->  micro_ros_agent
#
# The firmware (in Renode) runs the canonical wait-for-agent loop (rmw_uros_ping_agent),
# then rclc_support_init + rclc_node_init_default("stm32_node"). A real micro_ros_agent
# (Docker) bridged to Renode's USART2 must answer the ping, establish an XRCE session, and
# create the participant. PASS = the agent logs "session established" + "participant created".
#
# Two non-obvious requirements, both learned the hard way (see docs/TESTING.md):
#   1. Renode's CreateServerSocketTerminal defaults to a TELNET server which injects IAC
#      negotiation bytes and escapes 0xFF -> it mangles the binary XRCE stream. The third
#      arg `false` (emitConfigBytes=false) gives a RAW socket. Without it: no session.
#   2. The agent lives only in a Docker image, and a container cannot open a host pty
#      across the devpts namespace. So socat AND the agent run INSIDE one --network host
#      container (pty local to that container); the container reaches Renode on 127.0.0.1.
#      The image entrypoint is overridden to bash, and ROS is sourced by hand (otherwise
#      micro_ros_agent is not on PATH).
#
# Needs: renode, docker, and the micro-ros-agent image. The socat-augmented image
# (micro-ros-agent + socat) is built here on first run if absent. Exit 0 = round-trip OK.
set -u

ELF="${1:-$(cd "$(dirname "$0")/.." && pwd)/build/firmware_stm32.elf}"
RENODE="${RENODE:-}"
[ -z "$RENODE" ] && RENODE="$(command -v renode || true)"
[ -z "$RENODE" ] && [ -x /home/claude/renode-portable/renode ] && RENODE=/home/claude/renode-portable/renode
PORT="${UROS_PORT:-3401}"
BASE_IMG="${UROS_AGENT_IMG:-microros/micro-ros-agent:jazzy}"
AGENT_IMG=mr-agent-socat
RUN_SECS="${UROS_RUN_SECS:-45}"

[ -x "$RENODE" ] || { echo "ERROR: Renode not found (set \$RENODE)"; exit 2; }
[ -f "$ELF" ]    || { echo "ERROR: ELF not found: $ELF (run 'make build-fw')"; exit 2; }
command -v docker >/dev/null || { echo "ERROR: docker not found"; exit 2; }

# --- ensure the socat-augmented agent image exists -------------------------------------
if ! docker image inspect "$AGENT_IMG" >/dev/null 2>&1; then
  echo "=== building $AGENT_IMG ($BASE_IMG + socat) ==="
  docker build -t "$AGENT_IMG" - >/dev/null <<EOF
FROM $BASE_IMG
RUN apt-get update && apt-get install -y --no-install-recommends socat && rm -rf /var/lib/apt/lists/*
EOF
fi

LOG_AGENT=$(mktemp); LOG_RENODE=$(mktemp); RESC=$(mktemp --suffix=.resc)
cleanup() { pkill -f 'Renode.exe' 2>/dev/null
            docker ps -q --filter ancestor="$AGENT_IMG" | xargs -r docker stop >/dev/null 2>&1
            rm -f "$RESC"; }
trap cleanup EXIT

pkill -f 'Renode.exe' 2>/dev/null; sleep 1
# Pick a genuinely free port: a just-killed Renode leaves its socket in TIME_WAIT and
# Renode's CreateServerSocketTerminal has no SO_REUSEADDR, so reusing the same port back-to-
# back fails to bind. `ss -tan` lists TIME_WAIT too, so skip any port that appears at all.
for _ in $(seq 1 50); do
  ss -tan 2>/dev/null | grep -q ":$PORT " || break
  PORT=$((PORT+1))
done
echo "=== Renode: USART2 -> RAW TCP socket :$PORT, start (held ${RUN_SECS}s) ==="
cat > "$RESC" <<EOF
mach create "f446"
machine LoadPlatformDescription @platforms/cpus/stm32f4.repl
sysbus LoadELF @$ELF
emulation CreateServerSocketTerminal $PORT "urosterm" false
connector Connect sysbus.usart2 urosterm
start
EOF
( cd "$(dirname "$RENODE")" && (sleep "$RUN_SECS"; echo quit) | timeout $((RUN_SECS+10)) \
    ./"$(basename "$RENODE")" --console --disable-xwt "$RESC" ) > "$LOG_RENODE" 2>&1 &

# PASSIVE listen check only — never actually connect. Renode's socket terminal serves a
# single client; an active probe (then socat connecting second) races its accept slot and
# socat fails ("Serial port not found"). `ss` reads the LISTEN state without connecting,
# so socat is the sole client.
LISTENING=0
for i in $(seq 1 20); do
  ss -ltn 2>/dev/null | grep -q ":$PORT " && { LISTENING=1; echo "  listening after ${i}s"; break; }
  sleep 1
done
if [ "$LISTENING" != 1 ]; then
  echo "❌ Renode never listened on :$PORT. Likely the port is busy (set UROS_PORT to a free one)."
  echo "--- ss -ltn | grep $PORT ---"; ss -ltn | grep "$PORT" || echo "(nothing on $PORT)"
  echo "--- renode log tail ---"; tail -n 15 "$LOG_RENODE"
  exit 1
fi

echo "=== agent container (--network host): socat pty<->tcp, then micro_ros_agent ==="
docker run --rm --network host --entrypoint bash "$AGENT_IMG" -c "
  source /opt/ros/jazzy/setup.bash
  source /uros_ws/install/local_setup.bash
  socat pty,link=/tmp/vpty,raw,echo=0 tcp:127.0.0.1:$PORT &
  sleep 3
  ros2 run micro_ros_agent micro_ros_agent serial --dev /tmp/vpty -b 115200 -v6
" > "$LOG_AGENT" 2>&1 &

sleep "$RUN_SECS"
cleanup; trap - EXIT

echo; echo "---- agent session signals ----"
grep -iE 'session established|create_client|participant created|create_participant' "$LOG_AGENT" | sed 's/\x1b\[[0-9;]*m//g'
echo "---- node name seen on the wire (hex 73 74 6D 33 32 = 'stm32') ----"
grep -iE '73 74 6D 33 32' "$LOG_AGENT" | head -1 | sed 's/\x1b\[[0-9;]*m//g'

if grep -qiE 'session established' "$LOG_AGENT" && grep -qiE 'participant created' "$LOG_AGENT"; then
  echo "✅ Ф3 PASS: live micro-ROS round-trip — XRCE session + participant created."
  exit 0
else
  echo "❌ Ф3 FAIL: no XRCE session/participant. Agent log tail:"; tail -n 15 "$LOG_AGENT" | sed 's/\x1b\[[0-9;]*m//g'
  exit 1
fi
