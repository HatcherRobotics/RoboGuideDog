#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace core {

    enum SensorType
    {
        SensorTypeUnknown = -1
        , SensorTypeBumper
        , SensorTypeCliff
        , SensorTypeSonar
        , SensorTypeDepthCamera
        , SensorTypeWallSensor
        , SensorTypeMagTapeDetector
    };

    // old ImpactSensorKind are deprecated:
    //    rpos::message::impact::ImpactSensorKind
    //    rpos::features::impact_sensor::ImpactSensorKind
    // their int values are:
    //    ImpactSensorKindBumper = 0
    //    ImpactSensorKindCliff = 1
    //    ImpactSensorKindSonar = 2
    //    ImpactSensorKindWallSensor = 3
    //
    // returns SensorTypeUnknown if no match
    RPOS_CORE_API SensorType deprecatedImpactSensorKindIntValueToCoreSensorType(int deprecatedImpactSensorKindIntValue);
    // returns -1 if no match
    RPOS_CORE_API int coreSensorTypeToDeprecatedImpactSensorKindIntValue(SensorType coreSensorType);

}}
