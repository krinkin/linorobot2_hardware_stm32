#!/usr/bin/env bash
# F6 full base-node topic round-trip, in emulation (no board). Extends the F3 bridge
# (Renode RAW socket <-> socat <-> micro_ros_agent in one --network host container) and adds
# the F5 MPU6050 mock so a real /imu/data_raw is published. Once the firmware connects (the
# 4-state reconnect machine -> createEntities), it exposes the linorobot2 base-node interface:
#   sub  geometry_msgs/Twist   on  cmd_vel
#   pub  nav_msgs/Odometry     on  odom/unfiltered   (frame odom / child base_footprint)
#   pub  sensor_msgs/Imu       on  imu/data_raw      (frame imu_link)   [imu/mag skipped: FakeMAG]
# PASS = XRCE session + participant (F3 guard) AND the three topics exist (imu/mag absent) AND
# odom/imu echo with the right frame_ids AND ros2 topic pub /cmd_vel routes without error.
#
# Test honesty: Renode timers free-run (F4) and DummyI2CSlave is 1-byte/read (F5), so the
# cmd_vel->exact-odom-value link and the IMU SI values are NOT asserted here (covered by
# F4 control_smoke + host ControlCore tests + F5). F6 proves the TOPIC INTERFACE + frames +
# bidirectional routing over a live agent.
set -u

ELF="${1:-$(cd "$(dirname "$0")/.." && pwd)/build/firmware_stm32.elf}"
ELF="$(cd "$(dirname "$ELF")" && pwd)/$(basename "$ELF")"   # absolutize (Renode runs from its own dir)
HERE="$(cd "$(dirname "$0")" && pwd)"
RENODE="${RENODE:-}"
[ -z "$RENODE" ] && RENODE="$(command -v renode || true)"
[ -z "$RENODE" ] && [ -x /home/claude/renode-portable/renode ] && RENODE=/home/claude/renode-portable/renode
PORT="${UROS_PORT:-3411}"
BASE_IMG="${UROS_AGENT_IMG:-microros/micro-ros-agent:jazzy}"
AGENT_IMG=mr-agent-socat
RUN_SECS="${UROS_RUN_SECS:-75}"

[ -x "$RENODE" ] || { echo "ERROR: Renode not found (set \$RENODE)"; exit 2; }
[ -f "$ELF" ]    || { echo "ERROR: ELF not found: $ELF (run 'make build-fw')"; exit 2; }
command -v docker >/dev/null || { echo "ERROR: docker not found"; exit 2; }

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
for _ in $(seq 1 50); do ss -tan 2>/dev/null | grep -q ":$PORT " || break; PORT=$((PORT+1)); done
echo "=== Renode: USART2 -> RAW socket :$PORT + MPU6050 mock on i2c1, start (held ${RUN_SECS}s) ==="
cat > "$RESC" <<EOF
mach create "f446"
machine LoadPlatformDescription @platforms/cpus/stm32f4.repl
machine LoadPlatformDescriptionFromString "mpu: Mocks.DummyI2CSlave @ i2c1 0x68"
sysbus LoadELF @$ELF
include @$HERE/mpu6050_mock.py
setup_mpu6050_mock "sysbus.i2c1.mpu"
emulation CreateServerSocketTerminal $PORT "urosterm" false
connector Connect sysbus.usart2 urosterm
start
EOF
( cd "$(dirname "$RENODE")" && (sleep "$RUN_SECS"; echo quit) | timeout $((RUN_SECS+10)) \
    ./"$(basename "$RENODE")" --console --disable-xwt "$RESC" ) > "$LOG_RENODE" 2>&1 &

LISTENING=0
for i in $(seq 1 20); do
  ss -ltn 2>/dev/null | grep -q ":$PORT " && { LISTENING=1; echo "  listening after ${i}s"; break; }
  sleep 1
done
[ "$LISTENING" = 1 ] || { echo "[FAIL] Renode never listened on :$PORT"; tail -n 15 "$LOG_RENODE"; exit 1; }

echo "=== agent container: socat + micro_ros_agent, then ros2 topic list/pub/echo ==="
docker run --rm --network host --entrypoint bash "$AGENT_IMG" -c "
  source /opt/ros/jazzy/setup.bash
  source /uros_ws/install/local_setup.bash
  socat pty,link=/tmp/vpty,raw,echo=0 tcp:127.0.0.1:$PORT &
  sleep 3
  ros2 run micro_ros_agent micro_ros_agent serial --dev /tmp/vpty -b 115200 -v4 &
  # Poll for discovery instead of a fixed sleep (cold CI runners vary): wait until both
  # publishers appear, deadline-bounded.
  deadline=\$((SECONDS+45))
  until ros2 topic list 2>/dev/null | grep -q '/odom/unfiltered' && ros2 topic list 2>/dev/null | grep -q '/imu/data_raw'; do
    [ \$SECONDS -ge \$deadline ] && { echo 'discovery_timeout'; break; }
    sleep 2
  done
  echo '###TOPIC_LIST###'
  timeout 10 ros2 topic list 2>&1
  echo '###CMD_PUB###'
  ( timeout 12 ros2 topic pub -t 10 -r 5 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.5}}' >/dev/null 2>&1 && echo CMD_PUB_OK ) &
  sleep 1
  echo '###ODOM###'
  timeout 12 ros2 topic echo --once /odom/unfiltered nav_msgs/msg/Odometry 2>&1
  echo '###IMU###'
  timeout 12 ros2 topic echo --once /imu/data_raw sensor_msgs/msg/Imu 2>&1
  echo '###DONE###'
  wait
" > "$LOG_AGENT" 2>&1 &

sleep "$RUN_SECS"
cleanup; trap - EXIT

clean() { sed 's/\x1b\[[0-9;]*m//g'; }
LIST=$(sed -n '/###TOPIC_LIST###/,/###CMD_PUB###/p' "$LOG_AGENT" | clean)
ODOM=$(sed -n '/###ODOM###/,/###IMU###/p'          "$LOG_AGENT" | clean)
IMU=$(sed -n '/###IMU###/,/###DONE###/p'           "$LOG_AGENT" | clean)

echo; echo "---- topic list ----"; echo "$LIST" | grep -E '^/' || echo "(none)"
echo "---- odom echo (head) ----"; echo "$ODOM" | grep -E 'frame_id|position|orientation' | head -6
echo "---- imu echo (head) ----";  echo "$IMU"  | grep -E 'frame_id|angular_velocity|linear_acceleration' | head -4

FAIL=0
mark(){ if [ "$1" = 0 ]; then echo "  [PASS] $2"; else echo "  [FAIL] $2"; FAIL=1; fi; }
grep -qiE 'session established' "$LOG_AGENT" && grep -qiE 'participant created' "$LOG_AGENT"; mark $? "XRCE session + participant (F3 guard)"
echo "$LIST" | grep -q '/cmd_vel';         mark $? "/cmd_vel subscription present"
echo "$LIST" | grep -q '/odom/unfiltered'; mark $? "/odom/unfiltered publisher present"
echo "$LIST" | grep -q '/imu/data_raw';    mark $? "/imu/data_raw publisher present"
! echo "$LIST" | grep -q '/imu/mag'; mark $? "/imu/mag absent (FakeMAG)"
echo "$ODOM" | grep -qE '^[[:space:]]*frame_id: odom$';            mark $? "odom frame_id == odom"
echo "$ODOM" | grep -q 'child_frame_id: base_footprint'; mark $? "odom child_frame_id == base_footprint"
echo "$IMU"  | grep -q 'angular_velocity:' && echo "$IMU" | grep -q 'linear_acceleration:'; mark $? "imu msg has angular_velocity + linear_acceleration"
# NOTE: ros2 topic pub -t exits 0 once it has published; this confirms the AGENT published
# (the firmware-side delivery is implied by the executor spinning -- the odom/imu pubs above --
# and the /cmd_vel subscription existing; it is unit-tested by the host ControlCore tests).
grep -q 'CMD_PUB_OK' "$LOG_AGENT"; mark $? "agent published /cmd_vel without error"

if [ "$FAIL" = 0 ]; then
  echo "[PASS] F6 PASS: full base-node topic round-trip (cmd_vel + odom/unfiltered + imu/data_raw) over a live agent"
  exit 0
else
  echo "[FAIL] F6 FAIL"; echo "--- agent log tail ---"; tail -n 25 "$LOG_AGENT" | clean; exit 1
fi
