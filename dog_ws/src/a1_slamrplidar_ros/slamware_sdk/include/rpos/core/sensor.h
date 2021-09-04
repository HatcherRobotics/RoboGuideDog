#pragma once

#include <rpos/core/rpos_core_config.h>
#include <rpos/core/sensor_type.h>

#define RPOS_CORE_SENSOR_INVALID_ID     (-1)

namespace rpos { namespace core {

    enum SensorMaskDataPermanencyType
    {
        SensorMaskDataPermanencyTypeUnknown = -1
        , SensorMaskDataPermanencyTypeOnce
        , SensorMaskDataPermanencyTypeAlways
    };

    struct RPOS_CORE_API DisabledSensorMaskData 
    {
        int id;
        bool isAlways;

        DisabledSensorMaskData();
    };

    struct RPOS_CORE_API SensorMaskCtrlData 
    {
        int id;
        bool isAlways;
        bool isEnabled;

        SensorMaskCtrlData();
    };

    struct RPOS_CORE_API SensorBriefInfo
    {
        int id;
        SensorType type;
        bool isEnabled;
        SensorMaskDataPermanencyType maskPermanencyType;

        SensorBriefInfo();
    };

}}
