/* POSIX time/sleep glue (FreeRTOS-backed) + a HAL-UART micro-ROS custom transport.
 * clock_gettime: required by rcutils/microxrcedds time; FreeRTOS-tick based (advances).
 * usleep: required by rclc_sleep_ms.
 * transport: blocking HAL UART on huart2 (USART2). The live agent round-trip runs on this
 * blocking transport; the utils' IT/DMA transports remain an optional future optimization. */
#include <time.h>
#include <unistd.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include <uxr/client/transport.h>

extern UART_HandleTypeDef huart2;

int clock_gettime(clockid_t clk, struct timespec *t) {
    (void)clk;
    uint32_t ms = (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
                  ? 0U : (uint32_t)xTaskGetTickCount();
    if (t) { t->tv_sec = ms / 1000U; t->tv_nsec = (long)(ms % 1000U) * 1000000L; }
    return 0;
}

int usleep(useconds_t usec) { vTaskDelay(pdMS_TO_TICKS(usec / 1000U)); return 0; }

bool cubemx_transport_open(struct uxrCustomTransport* t) { (void)t; return true; }
bool cubemx_transport_close(struct uxrCustomTransport* t) { (void)t; return true; }
size_t cubemx_transport_write(struct uxrCustomTransport* t, const uint8_t* buf, size_t len, uint8_t* err) {
    (void)t; (void)err; HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100); return len;
}
size_t cubemx_transport_read(struct uxrCustomTransport* t, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    (void)t; (void)err;
    return (HAL_UART_Receive(&huart2, buf, len, (uint32_t)timeout) == HAL_OK) ? len : 0;
}
