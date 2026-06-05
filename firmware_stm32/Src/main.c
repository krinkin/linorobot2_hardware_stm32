/* GUI-free F446RE micro-ROS firmware skeleton (hand-written; no CubeMX).
 * HAL_Init (HSI) + USART2 (PA2/PA3) + a FreeRTOS task that brings up micro-ROS.
 * HAL timebase is the DWT cycle counter so FreeRTOS owns SysTick with no conflict.
 * NOTE (Plan 4): Renode does not model DWT; for the agent round-trip move the HAL
 * timebase to a TIM / the FreeRTOS tick so HAL_UART timeouts fire. */
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

UART_HandleTypeDef huart2;

/* transport (defined in microros_glue.c) */
bool cubemx_transport_open(struct uxrCustomTransport*);
bool cubemx_transport_close(struct uxrCustomTransport*);
size_t cubemx_transport_write(struct uxrCustomTransport*, const uint8_t*, size_t, uint8_t*);
size_t cubemx_transport_read(struct uxrCustomTransport*, uint8_t*, size_t, int, uint8_t*);

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

static void uros_task(void* arg) {
    (void)arg;
    rmw_uros_set_custom_transport(true, (void*)&huart2,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_node_t node;
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "stm32_node", "", &support);

    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
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

    xTaskCreate(uros_task, "uros", 6144, NULL, 24, NULL);  /* 6144 words = 24 KB */
    vTaskStartScheduler();
    for (;;) {}
}
