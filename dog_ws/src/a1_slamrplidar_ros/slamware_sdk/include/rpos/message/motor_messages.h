#pragma once

namespace rpos { namespace message { namespace motor {

    struct MotorInfo
    {
        unsigned int motorId;
        unsigned int minSpeed;          // these speed is a relative value (usually a PWM value)
        unsigned int maxSpeed;          // for instance: on almost all rp devices, minSpeed is 0, maxSpeed is 1023, if the currentSpeed is 512:
        unsigned int currentSpeed;      //               the motor will be driven by a PWM signal with about 50% duty ratio
    };

}}}