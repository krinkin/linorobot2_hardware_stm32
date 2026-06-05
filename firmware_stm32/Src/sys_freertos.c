// Make newlib's heap thread-safe under FreeRTOS. There are two tasks: control_task
// allocates via C++ `new` -> malloc, while uros_task allocates
// via the rcl default allocator -> malloc. newlib's malloc is NOT reentrant unless
// __malloc_lock/__malloc_unlock are provided, so without these two tasks could
// corrupt the heap if one preempts the other mid-allocation. Suspending the
// scheduler (not disabling interrupts) is the standard, low-latency guard.
#include "FreeRTOS.h"
#include "task.h"
#include <errno.h>
#include <sys/types.h>
#include <stdint.h>

struct _reent;

// Stack-overflow hook (required by configCHECK_FOR_STACK_OVERFLOW=2). Latches a
// Renode-observable flag and stops, so an undersized task stack becomes a visible symbol
// instead of a silent HardFault. renode/control_smoke.sh reads g_dbg_stack_overflow and
// fails if it is non-zero.
volatile unsigned long g_dbg_stack_overflow = 0;
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask; (void)pcTaskName;
    g_dbg_stack_overflow = 1;
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

void __malloc_lock(struct _reent *r)
{
    (void)r;
    vTaskSuspendAll();
}

void __malloc_unlock(struct _reent *r)
{
    (void)r;
    (void)xTaskResumeAll();
}

// Bounded _sbrk overriding the unbounded one in libnosys. The newlib heap (used by
// the rcl default allocator and by C++ operator new) grows up from `end` toward the
// top-of-RAM MSP/IRQ stack at `_estack`. The stock nosys _sbrk never checks that
// limit, so on exhaustion it hands out addresses into the stack -> silent corruption
// instead of a NULL from malloc (which RCCHECK could observe). Reserve a margin for
// the MSP/IRQ stack and fail cleanly with ENOMEM. Called only from malloc, which
// already holds __malloc_lock, so the static heap_end needs no extra guard.
extern char end[];      /* heap base   (linker) — array form avoids -Warray-bounds on the */
extern char _estack[];  /* top of RAM  (linker)    pointer arithmetic below */
#define LINO_STACK_RESERVE 0x800u   /* >= _Min_Stack_Size */

void *_sbrk(ptrdiff_t incr)
{
    static char *heap_end = end;
    /* integer arithmetic on the linker symbol -> no -Warray-bounds on a fake array */
    char *limit = (char *)((uintptr_t)_estack - LINO_STACK_RESERVE);
    if (incr > 0 && (heap_end + incr > limit || heap_end + incr < heap_end)) {
        errno = ENOMEM;
        return (void *)-1;
    }
    char *prev = heap_end;
    heap_end += incr;
    return prev;
}
