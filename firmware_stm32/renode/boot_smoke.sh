#!/usr/bin/env bash
# Renode boot smoke (F2): boot the firmware ELF and prove it comes ALIVE -- not merely
# "didn't HardFault". The strong signal is that USART2 emits the micro-ROS ping: that
# can only happen if the FreeRTOS scheduler started, uros_task was scheduled, and the
# HAL-UART transport ran. A startup hang (e.g. a configASSERT spin) emits nothing.
#
# Why not just check "PC != HardFault_Handler": a configASSERT(uxPriority<configMAX_PRIORITIES)
# failure disables interrupts and spins in an infinite self-branch -- NOT a HardFault -- so the
# old PC check passed a firmware that never started the scheduler. The ping-TX check catches it.
#
# Usage: boot_smoke.sh [path/to/firmware.elf]
# Renode binary: $RENODE, else `renode` on PATH, else the portable build. Needs socat.
set -u
ELF="${1:-$(dirname "$0")/../build/firmware_stm32.elf}"
ELF="$(cd "$(dirname "$ELF")" && pwd)/$(basename "$ELF")"   # absolutize
RENODE="${RENODE:-}"
[ -z "$RENODE" ] && RENODE="$(command -v renode || true)"
[ -z "$RENODE" ] && [ -x /home/claude/renode-portable/renode ] && RENODE=/home/claude/renode-portable/renode
[ -z "$RENODE" ] && { echo "ERROR: Renode not found (set \$RENODE or install renode)"; exit 2; }
[ -f "$ELF" ] || { echo "ERROR: ELF not found: $ELF (run 'make build-fw' first)"; exit 2; }
command -v socat >/dev/null || { echo "ERROR: socat not found (needed to read USART2)"; exit 2; }

PORT="${UROS_PORT:-3403}"
SECS="${BOOT_SECS:-12}"
BYTES=$(mktemp); RENLOG=$(mktemp); RESC=$(mktemp --suffix=.resc)
cleanup(){ pkill -f 'Renode.exe' 2>/dev/null; rm -f "$RESC"; }
trap 'cleanup' EXIT

pkill -f 'Renode.exe' 2>/dev/null; sleep 1
# Pick a genuinely free port: a just-killed Renode leaves its socket in TIME_WAIT and
# CreateServerSocketTerminal has no SO_REUSEADDR, so back-to-back reuse fails to bind.
for _ in $(seq 1 50); do
  ss -tan 2>/dev/null | grep -q ":$PORT " || break
  PORT=$((PORT+1))
done
# RAW socket terminal (emitConfigBytes=false): no telnet IAC bytes to pollute the capture.
cat > "$RESC" <<EOF
mach create "f446"
machine LoadPlatformDescription @platforms/cpus/stm32f4.repl
sysbus LoadELF @$ELF
emulation CreateServerSocketTerminal $PORT "fwterm" false
connector Connect sysbus.usart2 fwterm
start
EOF
( cd "$(dirname "$RENODE")" && (sleep "$SECS"; echo quit) | timeout $((SECS+10)) \
    ./"$(basename "$RENODE")" --console --disable-xwt "$RESC" ) > "$RENLOG" 2>&1 &

# PASSIVE listen check (ss) -- do NOT open a probe connection; Renode's socket terminal
# serves one client, so let our socat be that client.
for i in $(seq 1 20); do
  ss -ltn 2>/dev/null | grep -q ":$PORT " && break
  sleep 1
done
timeout $((SECS-4)) socat -u tcp:127.0.0.1:$PORT - > "$BYTES" 2>/dev/null
cleanup; trap - EXIT

NBYTES=$(wc -c < "$BYTES")
echo "boot smoke: USART2 emitted $NBYTES bytes in $((SECS-4))s"
if [ "$NBYTES" -gt 0 ]; then
  echo "  first bytes: $(head -c 16 "$BYTES" | xxd -p)"
  rm -f "$BYTES" "$RENLOG"
  echo "[PASS] F2 PASS: scheduler + uros_task + UART transport alive (firmware transmits the micro-ROS ping)"
  exit 0
fi

# --- no bytes: diagnose HardFault vs. a startup spin -------------------------------------
echo "[FAIL] F2 FAIL: USART2 silent -- firmware never reached the transmit path."
FAULT=$(arm-none-eabi-nm "$ELF" 2>/dev/null | awk '/ HardFault_Handler$/{print "0x"toupper($1)}' | head -n1)
RESC2=$(mktemp --suffix=.resc)
cat > "$RESC2" <<EOF
mach create "f446"
machine LoadPlatformDescription @platforms/cpus/stm32f4.repl
sysbus LoadELF @$ELF
emulation RunFor "1.0"
echo "PCVAL:"
cpu PC
quit
EOF
OUT=$(cd "$(dirname "$RENODE")" && timeout 60 ./"$(basename "$RENODE")" --console --disable-xwt "$RESC2" 2>&1)
rm -f "$RESC2"
PC=$(echo "$OUT" | grep -A1 'PCVAL:' | tail -n1 | tr -d ' \r')
echo "  stuck PC=$PC  (HardFault_Handler=$FAULT)"
SYM=$(arm-none-eabi-nm -n "$ELF" 2>/dev/null | awk -v pc=$((PC)) '{a=strtonum("0x"$1); if(a<=pc)p=$3; else{print p; exit}}')
echo "  nearest symbol: ${SYM:-?}  (a spin in xTaskCreate/prvInit => configASSERT priority/stack)"
rm -f "$BYTES" "$RENLOG"
exit 1
