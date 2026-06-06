#!/usr/bin/env bash
# Renode F4 control smoke: prove the encoder/PWM control loop comes alive and that an
# encoder count genuinely propagates through the whole chain -- in emulation, no board.
#
#  (a) CONFIG SMOKE: boot, run virtual time, assert the FreeRTOS scheduler ticked
#      (xTickCount > 0), the control task ran (g_dbg_ticks > 0 -> HAL_TIM_Encoder_Init +
#      HAL_TIM_PWM_Start + the 50 Hz loop executed without faulting), CPU not in HardFault.
#  (b) INJECTED-ENCODER (load-bearing): Renode's generic STM32_Timer FREE-RUNS its counter on
#      virtual time, which by itself would make RPM/odom nonzero. So we first STOP both wheel
#      encoder timers (TIM2=M1, TIM3=M2: clear CR1.CEN) and zero them, let getRPM settle to 0,
#      capture odom_x, then inject ONLY an M1 count ramp (TIM2->CNT). With both timers frozen,
#      the only thing that can move odometry is our injection -> assert odom_x CHANGED.
#
# NOT asserted (hardware-only, like F2/F3): real x4 quadrature counting, direction decode,
# and real PWM duty/waveform -- Renode models timer registers, not pin-level signals.
set -u
ELF="${1:-$(dirname "$0")/../build/firmware_stm32.elf}"
ELF="$(cd "$(dirname "$ELF")" && pwd)/$(basename "$ELF")"
RENODE="${RENODE:-}"
[ -z "$RENODE" ] && RENODE="$(command -v renode || true)"
[ -z "$RENODE" ] && [ -x /home/claude/renode-portable/renode ] && RENODE=/home/claude/renode-portable/renode
[ -z "$RENODE" ] && { echo "ERROR: Renode not found (set \$RENODE)"; exit 2; }
[ -f "$ELF" ] || { echo "ERROR: ELF not found: $ELF (run 'make build-fw')"; exit 2; }

addr() { arm-none-eabi-nm "$ELF" | awk -v s=" $1\$" '$0 ~ s {print "0x"$1; exit}'; }
A_TICKS=$(addr g_dbg_ticks)
A_RPM0=$(addr g_dbg_rpm)             # float[4]; [0] == M1 (TIM2)
A_ODX=$(addr g_dbg_odom_x)
A_XTICK=$(addr xTickCount)
A_SOV=$(addr g_dbg_stack_overflow)
FAULT=$(arm-none-eabi-nm "$ELF" | awk '/ HardFault_Handler$/{print "0x"toupper($1); exit}')
# TIM2 (M1) base 0x40000000, TIM3 (M2) base 0x40000400; CR1 at +0x00, CNT at +0x24.
TIM2_CR1=0x40000000; TIM2_CNT=0x40000024
TIM3_CR1=0x40000400; TIM3_CNT=0x40000424
[ -n "$A_TICKS" ] && [ -n "$A_RPM0" ] && [ -n "$A_ODX" ] || { echo "ERROR: debug symbols not found"; exit 2; }

pkill -f 'Renode.exe' 2>/dev/null; sleep 1
RESC=$(mktemp --suffix=.resc)
cat > "$RESC" <<EOF
mach create "f446"
machine LoadPlatformDescription @platforms/cpus/stm32f4.repl
sysbus LoadELF @$ELF
emulation RunFor "2.0"
echo "XTICK:"
sysbus ReadDoubleWord $A_XTICK
echo "TICKS:"
sysbus ReadDoubleWord $A_TICKS
echo "PC:"
cpu PC
sysbus WriteDoubleWord $TIM2_CR1 0
sysbus WriteDoubleWord $TIM2_CNT 0
sysbus WriteDoubleWord $TIM3_CR1 0
sysbus WriteDoubleWord $TIM3_CNT 0
emulation RunFor "0.3"
echo "ODX_BEFORE:"
sysbus ReadDoubleWord $A_ODX
sysbus WriteDoubleWord $TIM2_CNT 100000
emulation RunFor "0.1"
sysbus WriteDoubleWord $TIM2_CNT 250000
emulation RunFor "0.1"
sysbus WriteDoubleWord $TIM2_CNT 450000
emulation RunFor "0.1"
echo "ODX_AFTER:"
sysbus ReadDoubleWord $A_ODX
echo "SOV:"
sysbus ReadDoubleWord $A_SOV
EOF
OUT=$(cd "$(dirname "$RENODE")" && (sleep 30; echo quit) | timeout 70 \
        ./"$(basename "$RENODE")" --console --disable-xwt "$RESC" 2>&1)
rm -f "$RESC"; pkill -f 'Renode.exe' 2>/dev/null

after()  { echo "$OUT" | grep -A1 "$1" | tail -n1 | tr -d ' \r'; }
nz()     { [ -n "$1" ] && [ "$1" != "0x00000000" ] && [ "$1" != "0x0" ]; }
XTICK=$(after 'XTICK:'); TICKS=$(after 'TICKS:'); PC=$(after 'PC:')
ODXB=$(after 'ODX_BEFORE:'); ODXA=$(after 'ODX_AFTER:'); SOV=$(after 'SOV:')

echo "config-smoke : xTickCount=$XTICK  g_dbg_ticks=$TICKS  PC=$PC (HardFault=$FAULT)"
echo "injected-enc : odom_x (frozen encoders) $ODXB --inject M1 CNT ramp--> $ODXA"

FAIL=0
nz "$XTICK"          || { echo "  [FAIL] scheduler never ticked";   FAIL=1; }
nz "$TICKS"          || { echo "  [FAIL] control loop never ran";   FAIL=1; }
[ -n "$PC" ] && [ $((PC)) -ne $((FAULT)) ] || { echo "  [FAIL] CPU in HardFault"; FAIL=1; }   # numeric (case/zero-pad safe)
! nz "$SOV" || { echo "  [FAIL] FreeRTOS stack overflow (g_dbg_stack_overflow != 0)"; FAIL=1; }
[ -n "$ODXB" ] && [ -n "$ODXA" ] && [ "$ODXB" != "$ODXA" ] \
    || { echo "  [FAIL] injected encoder count did NOT move odometry (CNT->getRPM->odom not wired)"; FAIL=1; }

if [ "$FAIL" = 0 ]; then
  echo "[PASS] F4 PASS: control loop alive; injected encoder CNT moves getRPM -> odometry end-to-end"
  exit 0
else
  echo "[FAIL] F4 FAIL"; echo "$OUT" | tail -n 20; exit 1
fi
