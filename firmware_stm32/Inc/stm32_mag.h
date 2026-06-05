// Native STM32 magnetometer drivers (replaces the Arduino-bound default_mag.h). Plan 6
// ships only the verbatim FakeMAG fallback; real magnetometers (HMC5883L / AK8963 /
// QMC5883L) are added later behind the same USE_*_MAG selector (mirror of mag.h).
#ifndef STM32_MAG_H
#define STM32_MAG_H

#include "Arduino.h"
#include <micro_ros_utilities/string_utilities.h>
#include "mag_interface.h"

// Verbatim from firmware/lib/imu/default_mag.h — the no-MAG fallback.
class FakeMAG : public MAGInterface
{
public:
    FakeMAG() {}
    bool startSensor() override { return true; }
    geometry_msgs__msg__Vector3 readMagnetometer() override { return mag_; }
private:
    geometry_msgs__msg__Vector3 mag_{};
};

#ifndef MAG
#define USE_FAKE_MAG
#define MAG FakeMAG
#endif

#endif // STM32_MAG_H
