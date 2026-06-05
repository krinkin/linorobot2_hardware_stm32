// Host-only shim: just enough of the Arduino API for the portable libs to
// compile under g++. NOT used in any firmware build.
#ifndef HOST_TEST_ARDUINO_SHIM_H
#define HOST_TEST_ARDUINO_SHIM_H

#include <math.h>   // fabs, cos, sin
#include <stdint.h>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

// Arduino's constrain is a macro and works with mixed float/double args
// (pid.cpp calls constrain(double, float, float)); keep it a macro.
#ifndef constrain
#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

#endif // HOST_TEST_ARDUINO_SHIM_H
