/* GUI-free F446RE micro-ROS firmware (hand-written; no CubeMX).
 * HAL_Init (HSI) + USART2 (PA2/PA3) + two FreeRTOS tasks: uros_task (micro-ROS base node)
 * and control_task (50 Hz encoder/PID/motor/odom/IMU loop). The HAL timebase is backed by
 * the FreeRTOS tick (SysTick), so FreeRTOS stays the sole SysTick owner with no conflict —
 * and it works in Renode, which does not model the DWT cycle counter. */
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "control_loop.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_utilities/string_utilities.h>
#include <time.h>
#include <math.h>

UART_HandleTypeDef huart2;

/* transport (defined in microros_glue.c) */
bool cubemx_transport_open(struct uxrCustomTransport*);
bool cubemx_transport_close(struct uxrCustomTransport*);
size_t cubemx_transport_write(struct uxrCustomTransport*, const uint8_t*, size_t, uint8_t*);
size_t cubemx_transport_read(struct uxrCustomTransport*, uint8_t*, size_t, int, uint8_t*);

/* defined in microros_glue.c (FreeRTOS-tick based); newlib <time.h> gates the POSIX decl. */
int clock_gettime(clockid_t clk, struct timespec* t);

/* HAL timebase backed by the FreeRTOS tick (SysTick). Works in Renode (which does NOT
 * model the DWT cycle counter) and on hardware, and keeps FreeRTOS as the sole SysTick
 * owner. Before the scheduler starts, a free-running counter keeps any early HAL timeout
 * progressing (configTICK_RATE_HZ = 1000 -> tick == ms). */
static volatile uint32_t s_boot_ticks;
HAL_StatusTypeDef HAL_InitTick(uint32_t prio) { (void)prio; return HAL_OK; }
uint32_t HAL_GetTick(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) return (uint32_t)xTaskGetTickCount();
    return ++s_boot_ticks;
}

void HAL_UART_MspInit(UART_HandleTypeDef* h) {
    if (h->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitTypeDef g = {0};
        g.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        g.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &g);
    }
}

/* ---- micro-ROS base node (the comms half of firmware.ino's controlCallback) ----
 * uros_task owns the rclc executor and runs ONLY communication; the control loop runs
 * independently on control_task (deadman-safe even when the agent is down). Cross-task data
 * crosses ONLY through the plain-C control_loop snapshot surface. A 4-state reconnect machine
 * (ported from firmware.ino) destroys + recreates the entities on agent loss. */
/* assign-to-temp suppresses the rcl warn_unused_result attribute */
#define RCCHECK(fn)     { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) return false; }
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = (fn); (void)_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile uint32_t _last = 0; \
    uint32_t _now = HAL_GetTick(); if ((uint32_t)(_now - _last) > (MS)) { X; _last = _now; } } while (0)

enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
volatile int g_dbg_uros_state = WAITING_AGENT;   /* Renode-observable */
/* ROS-epoch offset (ns) computed from rmw_uros_sync_session; the local clock_gettime is
 * only the FreeRTOS tick (uptime), so header stamps must add this to reach wall-clock time.
 * Session-scoped -> recomputed on every (re)connect. */
static volatile int64_t s_time_offset_ns = 0;

static rclc_support_t   support;
static rcl_allocator_t  allocator;
static rcl_node_t       node;
static rclc_executor_t  executor;
static rcl_timer_t      control_timer;
static rcl_subscription_t twist_subscriber;
static rcl_publisher_t  odom_publisher;
static rcl_publisher_t  imu_publisher;
static geometry_msgs__msg__Twist     twist_msg;
static nav_msgs__msg__Odometry       odom_msg;
static sensor_msgs__msg__Imu         imu_msg;
static bool s_frames_set = false;

static void twistCallback(const void* msgin) {
    const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*)msgin;
    control_set_cmd((float)m->linear.x, (float)m->linear.y, (float)m->angular.z);
}

/* 50 Hz publish-only timer (control_task already did moveBase). Reads the snapshot surface,
 * assembles odom + imu in C, publishes. Fires inside the executor spin, so only when connected. */
static void controlPublishCallback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    float x, y, h, vx, vy, wz;
    control_get_odom(&x, &y, &h, &vx, &vy, &wz);
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    float qw = cosf(h * 0.5f), qz = sinf(h * 0.5f);   /* yaw-only quaternion */
    odom_msg.pose.pose.orientation.w = qw;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = qz;
    odom_msg.twist.twist.linear.x  = vx;
    odom_msg.twist.twist.linear.y  = vy;
    odom_msg.twist.twist.angular.z = wz;

    control_get_imu(&imu_msg);   /* numeric fields only; header.frame_id stays "imu_link" */
#ifdef USE_FAKE_IMU
    imu_msg.angular_velocity.z = odom_msg.twist.twist.angular.z;   /* fake gyro-z := yaw rate */
#endif
    /* MPU6050 (and FakeIMU) have no orientation estimate: publish the identity quaternion and
     * flag orientation as unavailable per REP-145 (cov[0] = -1), not an invalid all-zero quat. */
    imu_msg.orientation.w = 1.0; imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0; imu_msg.orientation.z = 0.0;
    imu_msg.orientation_covariance[0] = -1.0;

    /* Wall-clock stamp = local tick time + the ROS-epoch offset from session sync. */
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    int64_t ns = (int64_t)t.tv_sec * 1000000000LL + (int64_t)t.tv_nsec + s_time_offset_ns;
    odom_msg.header.stamp.sec = (int32_t)(ns / 1000000000LL);  odom_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
    imu_msg.header.stamp.sec  = odom_msg.header.stamp.sec;     imu_msg.header.stamp.nanosec  = odom_msg.header.stamp.nanosec;

    RCSOFTCHECK(rcl_publish(&imu_publisher,  &imu_msg,  NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

static bool createEntities(void) {
    allocator = rcl_get_default_allocator();
    executor  = rclc_executor_get_zero_initialized_executor();
    node      = rcl_get_zero_initialized_node();
    control_timer    = rcl_get_zero_initialized_timer();
    twist_subscriber = rcl_get_zero_initialized_subscription();
    odom_publisher   = rcl_get_zero_initialized_publisher();
    imu_publisher    = rcl_get_zero_initialized_publisher();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "stm32_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered"));
    RCCHECK(rclc_publisher_init_default(&imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
    RCCHECK(rclc_subscription_init_default(&twist_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_timer_init_default2(&control_timer, &support, RCL_MS_TO_NS(20), controlPublishCallback, true));
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    if (!s_frames_set) {   /* set-once: micro_ros_string_utilities_set heap-allocates; re-set leaks */
        odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
        odom_msg.child_frame_id  = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_footprint");
        imu_msg.header.frame_id  = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");
        odom_msg.pose.covariance[0]  = 1e-4; odom_msg.pose.covariance[7]  = 1e-4; odom_msg.pose.covariance[35]  = 1e-4;
        odom_msg.twist.covariance[0] = 1e-5; odom_msg.twist.covariance[7] = 1e-5; odom_msg.twist.covariance[35] = 1e-5;
        s_frames_set = true;
    }
    /* Time sync: compute the ROS-epoch offset so header stamps are wall-clock, not uptime. */
    if (rmw_uros_sync_session(1000) == RMW_RET_OK && rmw_uros_epoch_synchronized())
        s_time_offset_ns = (int64_t)rmw_uros_epoch_nanos() - (int64_t)HAL_GetTick() * 1000000LL;
    return true;
}

static void destroyEntities(void) {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    /* frame_id Strings are set-once and deliberately NOT fini'd here (reused across reconnects). */
}

static void uros_task(void* arg) {
    (void)arg;
    rmw_uros_set_custom_transport(true, (void*)&huart2,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

    for (;;) {
        switch (g_dbg_uros_state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500,
                g_dbg_uros_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            g_dbg_uros_state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
            if (g_dbg_uros_state == WAITING_AGENT) destroyEntities();
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200,
                g_dbg_uros_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (g_dbg_uros_state == AGENT_CONNECTED)
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            break;
        case AGENT_DISCONNECTED:
            control_set_cmd(0, 0, 0);   /* native fullStop; control_task deadman also enforces */
            destroyEntities();
            g_dbg_uros_state = WAITING_AGENT;
            break;
        default: break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));   /* yield so control_task (lower pri) never starves */
    }
}

/* Control task: brings up the encoder/motor HAL and runs the 50 Hz control loop
 * (PID over encoder feedback -> motor PWM, odometry, 200 ms deadman). Runs on its
 * own FreeRTOS task, independent of the micro-ROS agent, so the base is governed
 * even when comms are down. uros_task bridges /cmd_vel into control_set_cmd and
 * publishes /odom/unfiltered + /imu/data_raw from the snapshot surface. */
static void control_task(void* arg) {
    (void)arg;
    control_loop_init();
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        control_loop_tick();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));   /* 50 Hz */
    }
}

int main(void) {
    HAL_Init();                 /* stays on HSI 16 MHz (no PLL for first bring-up) */

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    /* Priority MUST be < configMAX_PRIORITIES (raw FreeRTOS, not CMSIS-RTOS where
     * osPriorityNormal==24). Tie it to the config so it can never exceed the bound
     * and trip configASSERT(uxPriority < configMAX_PRIORITIES) -> silent hang. */
    xTaskCreate(uros_task, "uros", 6144, NULL, configMAX_PRIORITIES - 2, NULL);  /* 6144 words = 24 KB */
    xTaskCreate(control_task, "ctrl", 2048, NULL, configMAX_PRIORITIES - 3, NULL); /* 2048 words = 8 KB */
    vTaskStartScheduler();
    for (;;) {}
}
