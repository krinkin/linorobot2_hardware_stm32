#!/usr/bin/env bash
# Renode F5 IMU smoke: prove the firmware drives an MPU6050 over the REAL HAL I2C path in
# emulation. A Python MPU6050 mock (Mocks.DummyI2CSlave @ i2c1 0x68, renode/mpu6050_mock.py)
# answers the firmware's HAL_I2C transactions, so this chain is exercised end-to-end:
#   HAL_I2C_Mem_Read/Write -> Mpu6050Imu (WHO_AM_I + wake + config + accel/gyro bursts) ->
#   imu_math -> sensor_msgs__msg__Imu.
#
# PASS criteria (what emulation can faithfully prove):
#   g_dbg_imu_ok != 0   -- startSensor() read WHO_AM_I==0x68 AND wrote PWR_MGMT/CONFIG over real
#                          HAL I2C (the I2C bring-up, the dev7<<1 address shift, and the
#                          1-byte Mem_Read/Mem_Write paths all work against a live slave model).
#   PC not in HardFault_Handler.
#
# Informational only (NOT a gate): g_dbg_accel_z_milli / g_dbg_gyro_z_milli. The driver DOES
# issue the correct accel(0x3B)/gyro(0x43) 6-byte bursts (visible in the Renode log), but
# Renode's generic Mocks.DummyI2CSlave returns only ONE byte per master-read, so multi-byte
# burst DATA cannot be reproduced in emulation -> the SI values are a hardware-only check
# (like the F3 round-trip wire-timing). The pure LSB->SI math is covered by test_imu_math.cpp.
set -u
ELF="${1:-$(dirname "$0")/../build/firmware_stm32.elf}"
ELF="$(cd "$(dirname "$ELF")" && pwd)/$(basename "$ELF")"
HERE="$(cd "$(dirname "$0")" && pwd)"
RENODE="${RENODE:-}"
[ -z "$RENODE" ] && RENODE="$(command -v renode || true)"
[ -z "$RENODE" ] && [ -x /home/claude/renode-portable/renode ] && RENODE=/home/claude/renode-portable/renode
[ -z "$RENODE" ] && { echo "ERROR: Renode not found (set \$RENODE)"; exit 2; }
[ -f "$ELF" ] || { echo "ERROR: ELF not found: $ELF (run 'make build-fw')"; exit 2; }

addr() { arm-none-eabi-nm "$ELF" | awk -v s=" $1\$" '$0 ~ s {print "0x"$1; exit}'; }
A_OK=$(addr g_dbg_imu_ok)
A_ACC=$(addr g_dbg_accel_z_milli)
A_GYR=$(addr g_dbg_gyro_z_milli)
FAULT=$(arm-none-eabi-nm "$ELF" | awk '/ HardFault_Handler$/{print "0x"toupper($1); exit}')
[ -n "$A_OK" ] && [ -n "$A_ACC" ] && [ -n "$A_GYR" ] || { echo "ERROR: IMU debug symbols not found"; exit 2; }

pkill -f 'Renode.exe' 2>/dev/null; sleep 1
RESC=$(mktemp --suffix=.resc)
cat > "$RESC" <<EOF
mach create "f446"
machine LoadPlatformDescription @platforms/cpus/stm32f4.repl
machine LoadPlatformDescriptionFromString "mpu: Mocks.DummyI2CSlave @ i2c1 0x68"
sysbus LoadELF @$ELF
include @$HERE/mpu6050_mock.py
setup_mpu6050_mock "sysbus.i2c1.mpu"
emulation RunFor "3.5"
echo "IMU_OK:"
sysbus ReadDoubleWord $A_OK
echo "ACCEL_Z:"
sysbus ReadDoubleWord $A_ACC
echo "GYRO_Z:"
sysbus ReadDoubleWord $A_GYR
echo "PC:"
cpu PC
EOF
OUT=$(cd "$(dirname "$RENODE")" && (sleep 20; echo quit) | timeout 60 \
        ./"$(basename "$RENODE")" --console --disable-xwt "$RESC" 2>&1)
rm -f "$RESC"; pkill -f 'Renode.exe' 2>/dev/null

after()   { echo "$OUT" | grep -A1 "$1" | tail -n1 | tr -d ' \r'; }
todec()   { printf '%d' "$1" 2>/dev/null; }    # 0x.. (incl. two's-complement) -> signed-ish decimal
OK=$(after 'IMU_OK:'); ACC=$(after 'ACCEL_Z:'); GYR=$(after 'GYRO_Z:'); PC=$(after 'PC:')
ACC_D=$(todec "$ACC"); GYR_D=$(todec "$GYR")

echo "imu  : g_dbg_imu_ok=$OK  PC=$PC (HardFault=$FAULT)"
echo "info : accel_z_milli=$ACC ($ACC_D)  gyro_z_milli=$GYR ($GYR_D)  [burst data: hardware-only, see header]"

FAIL=0
[ -n "$OK" ] && [ "$OK" != "0x00000000" ] && [ "$OK" != "0x0" ] || { echo "  [FAIL] IMU init failed (WHO_AM_I/config not seen over real HAL I2C)"; FAIL=1; }
[ -n "$PC" ] && [ $((PC)) -ne $((FAULT)) ] || { echo "  [FAIL] CPU in HardFault"; FAIL=1; }   # numeric (case/zero-pad safe)

if [ "$FAIL" = 0 ]; then
  echo "[PASS] F5 PASS: MPU6050 detected + configured over real HAL I2C (WHO_AM_I + PWR_MGMT/CONFIG); no fault"
  exit 0
else
  echo "[FAIL] F5 FAIL"; echo "$OUT" | grep -iE 'i2c|mpu|error|exception|fault' | head -20; exit 1
fi
