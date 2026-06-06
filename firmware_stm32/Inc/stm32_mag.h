// Native STM32 magnetometer drivers. The port currently ships only the FakeMAG fallback;
// real magnetometers (HMC5883L / AK8963 / QMC5883L) can be added later behind the same
// USE_*_MAG selector.
#ifndef STM32_MAG_H
#define STM32_MAG_H

#include <micro_ros_utilities/string_utilities.h>
#include "mag_interface.h"

// The no-MAG fallback (same shape as the upstream linorobot2 FakeMAG).
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
