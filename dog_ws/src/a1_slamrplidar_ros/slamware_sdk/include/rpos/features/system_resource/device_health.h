/**
* device_health.h
* 
*
* Created   @ 2016-7-4
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <string>
#include <vector>
#include <cstdint>

#include <boost/optional.hpp>

namespace rpos { namespace features { namespace system_resource {
    enum BaseErrorLevel
    {
        BaseErrorLevelHealthy = 0,
        BaseErrorLevelWarn = 1,
        BaseErrorLevelError = 2,
        BaseErrorLevelFatal = 4,
        BaseErrorLevelUnknown = 255
    };

    enum BaseErrorComponent
    {
        BaseErrorComponentUser,
        BaseErrorComponentSystem,
        BaseErrorComponentPower,
        BaseErrorComponentMotion,
        BaseErrorComponentSensor,
        BaseErrorComponentUnknown = 255
    };

    enum BaseComponentErrorType
    {
        // Note: DO NOT remove anyone of the following items!!!
        //       JUST append to each segment.

        BaseComponentErrorTypeUnknown = -1
        
        // User
        , BaseComponentErrorTypeUser = (1024 * 1)

        // System
        , BaseComponentErrorTypeSystemNone = (1024 * 2) // dummy item, not used.
        , BaseComponentErrorTypeSystemEmergencyStop
        , BaseComponentErrorTypeSystemTemperatureHigh
        , BaseComponentErrorTypeSystemTemperatureLow
        , BaseComponentErrorTypeSystemWatchDogOverFlow
        , BaseComponentErrorTypeSystemCtrlBusDisconnected
        , BaseComponentErrorTypeSystemSlamwareRebooted
        , BaseComponentErrorTypeSystemBrakeReleased
        , BaseComponentErrorTypeSystemSlamwareRelocalizationFailed
        , BaseComponentErrorTypeSystemSlamwareLowLocalizationQuality

        // Power
        , BaseComponentErrorTypePowerNone = (1024 * 3) // dummy item, not used.
        , BaseComponentErrorTypePowerControllerDown
        , BaseComponentErrorTypePowerPowerLow
        , BaseComponentErrorTypePowerOverCurrent

        // Motion
        , BaseComponentErrorTypeMotionNone = (1024 * 4) // dummy item, not used.
        , BaseComponentErrorTypeMotionControllerDown
        , BaseComponentErrorTypeMotionMotorAlarm
        , BaseComponentErrorTypeMotionMotorDown
        , BaseComponentErrorTypeMotionOdometryDown
        , BaseComponentErrorTypeMotionBrushStall
        , BaseComponentErrorTypeMotionBlowerStall

        // Sensor
        , BaseComponentErrorTypeSensorNone = (1024 * 5) // dummy item, not used.
        , BaseComponentErrorTypeSensorControllerDown
        , BaseComponentErrorTypeSensorBumperDown
        , BaseComponentErrorTypeSensorCliffDown
        , BaseComponentErrorTypeSensorSonarDown
        , BaseComponentErrorTypeSensorDustbinBlock
        , BaseComponentErrorTypeSensorDustbinGone
        , BaseComponentErrorTypeSensorWallIrDown
        , BaseComponentErrorTypeSensorMagTapeTriggered
        , BaseComponentErrorTypeSensorMagSelfTestFailed
        , BaseComponentErrorTypeSensorIMUDown
    };

    struct RPOS_CORE_API BaseError
    {
    public:
        static const int INVALID_COMPONENT_ERROR_DEVICE_ID = -1;

        // Motion
        static const int MOTION_BRUSH_ID_ROLLING = 0;
        static const int MOTION_BRUSH_ID_LEFT_SIDE = 1;
        static const int MOTION_BRUSH_ID_RIGHT_SIDE = 2;

    public:
        int id;
        std::uint32_t errorCode;
        BaseErrorLevel level;
        BaseErrorComponent component;
        std::uint16_t componentErrorCode;
        std::string message;

        BaseComponentErrorType componentErrorType;
        int componentErrorDeviceId;

    public:
        BaseError();
    };

    struct RPOS_CORE_API BaseHealthInfo
    {
    public:
        bool hasWarning;
        bool hasError;
        bool hasFatal;

        std::vector<BaseError> errors;

        boost::optional<bool> hasSystemEmergencyStop;
        boost::optional<bool> hasLidarDisconnected;
        boost::optional<bool> hasSdpDisconnected;
        boost::optional<bool> hasDepthCameraDisconnected;

    public:
        BaseHealthInfo();
    };

} } }
