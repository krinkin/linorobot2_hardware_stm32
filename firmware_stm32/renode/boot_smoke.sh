#!/usr/bin/env bash
# Renode boot smoke (Ф2): boot the firmware ELF, run a slice of virtual time, and
# assert the CPU is alive and NOT stuck in HardFault_Handler. Exit 0 = pass.
# Usage: boot_smoke.sh [path/to/firmware.elf]
# Renode binary: $RENODE, else `renode` on PATH, else the portable build.
set -u
ELF="${1:-$(dirname "$0")/../build/firmware_stm32.elf}"
ELF="$(cd "$(dirname "$ELF")" && pwd)/$(basename "$ELF")"   # absolutize
RENODE="${RENODE:-}"
[ -z "$RENODE" ] && RENODE="$(command -v renode || true)"
[ -z "$RENODE" ] && [ -x /home/claude/renode-portable/renode ] && RENODE=/home/claude/renode-portable/renode
[ -z "$RENODE" ] && { echo "ERROR: Renode not found (set \$RENODE or install renode)"; exit 2; }
[ -f "$ELF" ] || { echo "ERROR: ELF not found: $ELF (run 'make build-fw' first)"; exit 2; }

FAULT=$(arm-none-eabi-nm "$ELF" | awk '/ HardFault_Handler$/{print "0x"$1}' | head -n1)
RESC=$(mktemp --suffix=.resc)
cat > "$RESC" <<EOF
mach create "f446"
machine LoadPlatformDescription @platforms/cpus/stm32f4.repl
sysbus LoadELF @$ELF
emulation RunFor "1.0"
echo "PCVAL:"
cpu PC
echo "INSTRVAL:"
cpu ExecutedInstructions
quit
EOF
OUT=$(cd "$(dirname "$RENODE")" && timeout 240 ./"$(basename "$RENODE")" --console --disable-xwt "$RESC" 2>&1)
rm -f "$RESC"
PC=$(echo "$OUT"    | grep -A1 'PCVAL:'    | tail -n1 | tr -d ' \r')
INSTR=$(echo "$OUT" | grep -A1 'INSTRVAL:' | tail -n1 | tr -d ' \r')
echo "boot smoke: PC=$PC  ExecutedInstructions=$INSTR  (HardFault_Handler=$FAULT)"
if [ -n "$PC" ] && [ "$PC" != "$FAULT" ] && [ "$INSTR" != "0x0000000000000000" ]; then
  echo "✅ Ф2 PASS: booted, FreeRTOS running, no HardFault"; exit 0
else
  echo "❌ Ф2 FAIL: CPU stuck/halted/faulted"; echo "$OUT" | tail -n 20; exit 1
fi
