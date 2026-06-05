// Verbatim copy of firmware/lib/motor/motor_interface.h (Apache-2.0, (c) 2021
// Juan Miguel Jimeno). Copied into the STM32 port so the build does NOT
// transitively pull in the Arduino-bound default_motor.h. The spin()/invert/sign
// dispatch is reused unchanged so motor behavior matches the Arduino firmware.
#ifndef MOTOR_INTERFACE
#define MOTOR_INTERFACE

class MotorInterface
{
    bool invert_;
    protected:
        virtual void forward(int pwm) = 0;
        virtual void reverse(int pwm) = 0;

    public:
        MotorInterface(int invert):
            invert_(invert)
        {
        }

        virtual void brake() = 0;
        void spin(int pwm)
        {
            if(invert_)
                pwm *= -1;

            if(pwm > 0)
                forward(pwm);
            else if(pwm < 0)
                reverse(pwm);
            else
                brake();
        }
};

#endif
