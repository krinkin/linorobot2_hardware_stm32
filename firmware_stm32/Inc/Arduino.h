// Firmware-side shim: just enough of the Arduino API for the portable libs
// (kinematics.cpp, pid.cpp) to compile in the native STM32 build. The portable
// headers #include "Arduino.h" for PI / constrain / fabs only — no I/O. (Mirror
// of test_host/shim/Arduino.h; that one is host-only, this one is firmware-only.)
#ifndef LINO_STM32_ARDUINO_SHIM_H
#define LINO_STM32_ARDUINO_SHIM_H

#include <math.h>     // fabs, cos, sin
#include <stdint.h>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

// Arduino's constrain is a macro that works with mixed float/double args
// (pid.cpp calls constrain(double, float, float)); keep it a macro.
#ifndef constrain
#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

#endif // LINO_STM32_ARDUINO_SHIM_H
