/*
* imu_messages.h
* IMU messages
*
* Was in i_imu_device.h
* Created by Tony Huang at 2016-11-15
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/core/geometry.h>

namespace rpos { namespace message { namespace imu {

    enum ImuAxis
    {
        ImuAxisX = 1,
        ImuAxisY = 2,
        ImuAxisZ = 4,
        ImuAxisNone = 0,
        ImuAxisAll = ImuAxisX | ImuAxisY | ImuAxisZ
    };

    enum ImuDataSet
    {
        ImuDataSetRaw = 1,
        ImuDataSetFusion = 2,
        ImuDataSetProcessed = 4,
        ImuDataSetNone = 0,
        ImuDataSetAll = ImuDataSetRaw | ImuDataSetFusion | ImuDataSetProcessed
    };

    enum ImuSensor
    {
        ImuSensorAcc,
        ImuSensorGyro,
        ImuSensorCompass
    };

    struct ImuDeviceInfo
    {
        ImuAxis gyroAxises;
        ImuDataSet gyroDataSets;
        ImuAxis accAxises;
        ImuDataSet accDataSets;
        ImuAxis compassAxises;
        ImuDataSet compassDataSets;
    };

    struct ImuAllSensorData
    {
        rpos::core::Vector3f acc;
        rpos::core::Vector3f gyro;
        rpos::core::Vector3f gyroIntegral;
        rpos::core::Vector3f compass;
    };

    typedef rpos::core::Vector3f ImuSensorData;

} } }
