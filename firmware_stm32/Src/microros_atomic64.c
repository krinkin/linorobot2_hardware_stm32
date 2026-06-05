/* 64-bit atomics shim for single-core Cortex-M (STM32F446, M4F).
 * REQUIRED: rcl (time/timer/client) references __atomic_*_8 unconditionally and
 * arm-none-eabi has no baremetal 64-bit atomics (no LDREXD; no libatomic).
 * RCUTILS_NO_64_ATOMIC=ON covers rcutils only. Without this, the F0 link fails on
 * "undefined reference to __atomic_load_8". Ref: micro_ros_stm32cubemx_utils#112.
 * SINGLE-CORE ONLY: mutual exclusion vs ISRs via PRIMASK (WRONG on dual-core H7). */
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_compiler.h"   /* __get_PRIMASK / __disable_irq / __set_PRIMASK */

static inline uint32_t cs_enter(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static inline void cs_exit(uint32_t p) { __set_PRIMASK(p); }

uint64_t __atomic_load_8(const volatile void *ptr, int mo) {
    (void)mo; uint32_t s = cs_enter(); uint64_t v = *(const volatile uint64_t*)ptr; cs_exit(s); return v;
}
void __atomic_store_8(volatile void *ptr, uint64_t val, int mo) {
    (void)mo; uint32_t s = cs_enter(); *(volatile uint64_t*)ptr = val; cs_exit(s);
}
uint64_t __atomic_exchange_8(volatile void *ptr, uint64_t val, int mo) {
    (void)mo; uint32_t s = cs_enter(); volatile uint64_t *p = ptr; uint64_t o = *p; *p = val; cs_exit(s); return o;
}
bool __atomic_compare_exchange_8(volatile void *ptr, void *exp, uint64_t des,
                                 bool weak, int smo, int fmo) {
    (void)weak; (void)smo; (void)fmo;
    uint32_t s = cs_enter(); volatile uint64_t *p = ptr; uint64_t *e = exp;
    bool ok = (*p == *e); if (ok) *p = des; else *e = *p; cs_exit(s); return ok;
}
uint64_t __atomic_fetch_add_8(volatile void *ptr, uint64_t val, int mo) {
    (void)mo; uint32_t s = cs_enter(); volatile uint64_t *p = ptr; uint64_t o = *p; *p = o + val; cs_exit(s); return o;
}
