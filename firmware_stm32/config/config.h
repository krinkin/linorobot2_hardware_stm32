// Board-config selector for the native STM32 port. The active board is chosen by a
// -DUSE_*_CONFIG build flag from the
// Makefile. C++-only: the board headers pull in lino_hal.h (HAL struct types), so
// C translation units (e.g. main.c) must NOT include this -- they talk to the
// control loop through the plain-C control_loop.h surface instead.
#ifndef LINO_STM32_CONFIG_H
#define LINO_STM32_CONFIG_H

#ifdef USE_F446RE_CONFIG
#include "f446re_config.h"
#endif

#ifndef LINO_BASE
#error "No board config selected: define USE_F446RE_CONFIG (or another USE_*_CONFIG)."
#endif

#endif // LINO_STM32_CONFIG_H
