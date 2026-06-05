# Renode Python I2C-slave mock of an MPU6050, for the Phase-5 IMU smoke test. Attaches
# to a Mocks.DummyI2CSlave at addr 0x68 and answers the real HAL_I2C_Mem_Read/Write the
# firmware issues, so the WHOLE native path (HAL I2C -> Mpu6050Imu -> imu_math -> Vector3)
# is exercised in emulation.
#
# HAL_I2C_Mem_Read(reg, buf, n): write phase delivers [reg] (len 1) -> we enqueue the
#   register bank slice the master then reads back.
# HAL_I2C_Mem_Write(reg, val):   write phase delivers [reg, val] (len 2) -> we store val.
class Mpu6050Mock:
    def __init__(self, dummy):
        self.dummy = dummy
        self.bank = [0] * 0x80
        self.bank[0x75] = 0x68                 # WHO_AM_I
        # ACCEL_XOUT 0x3B..0x40 (big-endian): ax=0, ay=0, az=+16384 (=0x4000 -> ~9.81 m/s^2)
        self.bank[0x3F] = 0x40; self.bank[0x40] = 0x00
        # GYRO_XOUT 0x43..0x48 (big-endian): gx=0, gy=0, gz=+131 (=0x0083 -> ~0.01745 rad/s)
        self.bank[0x47] = 0x00; self.bank[0x48] = 0x83

    def write(self, data):
        n = len(data)
        if n == 1:                             # Mem_Read: reg pointer, response follows
            reg = data[0] & 0x7F
            count = 6 if reg in (0x3B, 0x43) else 1   # accel/gyro bursts are 6 bytes
            for i in range(count):             # enqueue byte-by-byte (robust marshaling)
                self.dummy.EnqueueResponseByte(self.bank[(reg + i) & 0x7F])
        elif n >= 2:                           # Mem_Write: store config byte(s)
            self.bank[data[0] & 0x7F] = data[1]

def mc_setup_mpu6050_mock(path):
    dummy = monitor.Machine[path]
    dummy.DataReceived += Mpu6050Mock(dummy).write
