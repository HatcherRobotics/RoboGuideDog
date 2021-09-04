#pragma once

#include <rpos/message/message.h>
#include <rpos/core/pose.h>
#include <rpos/core/sensor_type.h>

namespace rpos { namespace message { namespace impact {

    enum ImpactSensorType
    {
        ImpactSensorTypeDigital,    // two value sensors, zero to FLT_EPSILON for impact, FLT_MAX for non-impact 
        ImpactSensorTypeAnalog      // analog value sensors, report distance in meter
    };

    typedef unsigned int impact_sensor_id_t;

    struct ImpactSensorInfo
    {
        impact_sensor_id_t id;
        rpos::core::Pose pose;
        ImpactSensorType type;
        rpos::core::SensorType coreSensorType; // deprecated member: ImpactSensorKind kind;
        float refreshFreq;
    };

    typedef float ImpactSensorValue;

    static const float ImpactSensorDigitalImpact = 0;
    static const float ImpactSensorDigitalNotImpact = FLT_MAX;

    static inline bool digitalIsImpact(ImpactSensorValue v)
    {
        return v < FLT_EPSILON;
    }

    struct ContactSensorData
    {
        ImpactSensorInfo info;
        ImpactSensorValue value;
    };

}}}