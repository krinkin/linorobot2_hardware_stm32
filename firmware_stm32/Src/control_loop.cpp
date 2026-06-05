// The native STM32 control module = the firmware.ino moveBase() equivalent.
// Owns 4 Encoder + 4 Motor + 4 PID + Kinematics + OdomIntegrator, constructed in
// control_loop_init() (NOT as global ctors — they must run AFTER HAL_Init so the
// clocks are up). Single-threaded: control runs on its own FreeRTOS task (see
// main.c), independent of the micro-ROS agent, so the motors are governed (and the
// deadman brakes) even when comms are down. The actual math is the host-tested
// ControlCore; HAL only appears via the Encoder/Motor adapters.
#include "config.h"            // board macros + USE_GENERIC_2... + USE_MPU6050_IMU + descriptor tables
#include "stm32_encoder.h"
#include "stm32_motor.h"       // defines `Motor` from USE_*_MOTOR_DRIVER
#include "stm32_i2c.h"
#include "stm32_imu.h"         // defines `IMU`  from USE_*_IMU
#include "stm32_mag.h"         // defines `MAG`  from USE_*_MAG
#include "control_core.h"
#include "control_loop.h"
#include "lino_hal.h"
#include "kinematics.h"
#include "pid.h"
#include "odom_integrator.h"

namespace {
Encoder*       enc[4];
Motor*         mot[4];
PID*           pid_[4];
Kinematics*    kin;
OdomIntegrator odom;
ControlCore*   core;
I2cBus*        i2c;
IMU*           imu;
MAG*           mag;

volatile float    cmd_x = 0, cmd_y = 0, cmd_wz = 0;
volatile uint32_t prev_cmd_ms = 0, prev_odom_ms = 0;
volatile float    last_vx = 0, last_vy = 0, last_wz = 0;

const uint32_t CMD_TIMEOUT_MS = 200;   // deadman
}

// Renode-observable debug symbols (read by name in renode/control_smoke.sh).
// Global linkage so they appear in the ELF symbol table.
extern "C" {
volatile float    g_dbg_rpm[4]       = {0, 0, 0, 0};
volatile float    g_dbg_odom_x       = 0;
volatile float    g_dbg_odom_y       = 0;
volatile float    g_dbg_odom_heading = 0;
volatile uint32_t g_dbg_ticks        = 0;
volatile uint32_t g_dbg_imu_ok        = 0;   // 1 once imu->init() succeeded (WHO_AM_I over real I2C)
volatile uint32_t g_dbg_i2c_init_ok   = 0;   // 1 if HAL_I2C_Init succeeded (distinct from a WHO_AM_I mismatch)
volatile uint32_t g_dbg_imu_read_ok   = 0;   // latest per-tick IMU register read succeeded (staleness signal)
volatile int32_t  g_dbg_accel_z_milli = 0;   // linear_acceleration.z * 1000 (m/s^2)
volatile int32_t  g_dbg_gyro_z_milli  = 0;   // angular_velocity.z   * 1000 (rad/s)
}

extern "C" void control_loop_init(void)
{
    lino_time_init();   // 1 MHz TIM5 micros base for the encoder RPM dt

    enc[0] = new Encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
    enc[1] = new Encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
    enc[2] = new Encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
    enc[3] = new Encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

    mot[0] = new Motor(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
    mot[1] = new Motor(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
    mot[2] = new Motor(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
    mot[3] = new Motor(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

    for (int i = 0; i < 4; ++i)
        pid_[i] = new PID(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

    kin = new Kinematics(Kinematics::LINO_BASE, MOTOR_MAX_RPM, MAX_RPM_RATIO,
                         MOTOR_OPERATING_VOLTAGE, MOTOR_POWER_MAX_VOLTAGE,
                         WHEEL_DIAMETER, LR_WHEELS_DISTANCE);

    for (int i = 0; i < 4; ++i) { enc[i]->init(); mot[i]->init(); }

    core = new ControlCore(*kin, odom, pid_[0], pid_[1], pid_[2], pid_[3],
                           mot[0], mot[1], mot[2], mot[3]);

    // IMU + MAG. The real MPU6050 needs the I2C bus; FakeIMU/FakeMAG need nothing.
    // init() runs WHO_AM_I + wake (and ~2 s gyro calibration); blocks this task at startup
    // only — no /cmd_vel yet, deadman holds the base stopped.
#ifdef USE_FAKE_IMU
    imu = new IMU();
    g_dbg_i2c_init_ok = 1u;   // no bus
#else
    i2c = new I2cBus(IMU_I2C_DESC);
    g_dbg_i2c_init_ok = i2c->init() ? 1u : 0u;
    imu = new IMU(i2c, MPU6050_I2C_ADDR);
#endif
    mag = new MAG();
    g_dbg_imu_ok = imu->init() ? 1u : 0u;
    mag->init();

    prev_cmd_ms = prev_odom_ms = lino_millis();
    cmd_x = cmd_y = cmd_wz = 0;
}

extern "C" void control_loop_tick(void)
{
    uint32_t now = lino_millis();

    // Snapshot the shared command as one atomic tuple (control_set_cmd, called from the
    // micro-ROS task in Plan 7, writes all four fields). Apply the deadman to the snapshot.
    float lx, ly, lwz; uint32_t last_cmd;
    taskENTER_CRITICAL();
    lx = cmd_x; ly = cmd_y; lwz = cmd_wz; last_cmd = prev_cmd_ms;
    taskEXIT_CRITICAL();
    if ((now - last_cmd) >= CMD_TIMEOUT_MS) { lx = ly = lwz = 0; }   // deadman: no /cmd_vel -> stop

    // One timestamp for all wheels so each RPM dt matches the odom dt exactly.
    uint32_t now_us = lino_micros();
    float rpm[4] = { enc[0]->getRPM(now_us), enc[1]->getRPM(now_us),
                     enc[2]->getRPM(now_us), enc[3]->getRPM(now_us) };

    float dt = (float)(now - prev_odom_ms) / 1000.0f;
    prev_odom_ms = now;

    Kinematics::velocities v = core->step(lx, ly, lwz, rpm, dt);

    // Publish the pose + body-velocity snapshot atomically for control_get_odom (Plan 7).
    taskENTER_CRITICAL();
    last_vx = v.linear_x; last_vy = v.linear_y; last_wz = v.angular_z;
    taskEXIT_CRITICAL();

    // Read the IMU (Plan 6 read path; publishing /imu/data_raw is Plan 7). Gated on a
    // successful init so a missing/faulted device never injects blocking I2C timeouts into
    // the 50 Hz loop every tick. Two ~0.6 ms bursts when present, well within the 20 ms budget.
    if (g_dbg_imu_ok) {
        sensor_msgs__msg__Imu im = imu->getData();
        g_dbg_imu_read_ok   = imu->readOk() ? 1u : 0u;
        g_dbg_accel_z_milli = (int32_t)(im.linear_acceleration.z * 1000.0);
        g_dbg_gyro_z_milli  = (int32_t)(im.angular_velocity.z   * 1000.0);
    }

    for (int i = 0; i < 4; ++i) g_dbg_rpm[i] = rpm[i];
    g_dbg_odom_x       = odom.x;
    g_dbg_odom_y       = odom.y;
    g_dbg_odom_heading = odom.heading;
    g_dbg_ticks++;
}

extern "C" void control_set_cmd(float linear_x, float linear_y, float angular_z)
{
    uint32_t now = lino_millis();
    taskENTER_CRITICAL();
    cmd_x = linear_x; cmd_y = linear_y; cmd_wz = angular_z;
    prev_cmd_ms = now;          // reset the deadman in the same atomic write
    taskEXIT_CRITICAL();
}

extern "C" void control_get_odom(float* x, float* y, float* heading,
                                 float* vx, float* vy, float* wz)
{
    taskENTER_CRITICAL();
    float ox = odom.x, oy = odom.y, oh = odom.heading;
    float lvx = last_vx, lvy = last_vy, lwz_ = last_wz;
    taskEXIT_CRITICAL();
    if (x)       *x = ox;
    if (y)       *y = oy;
    if (heading) *heading = oh;
    if (vx)      *vx = lvx;
    if (vy)      *vy = lvy;
    if (wz)      *wz = lwz_;
}
