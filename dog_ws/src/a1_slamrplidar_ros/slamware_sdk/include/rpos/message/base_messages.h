/*
* base_messages.h
* Base messages
*
* Was in i_base_device.h
* Created by Tony Huang (tony@slamtec.com) at 2016-11-15
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/


#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <rpos/core/geometry.h>

namespace rpos { namespace message { namespace base {

    enum BaseMotionModel
    {
        BaseMotionModelTwoWheelDifferential,
        BaseMotionModelThreeOmniWheel,
        // BaseMotionModelFourMcNameeWheel,
        // BaseMotionModelSyncTurnWheel
    };

    struct BaseInfo
    {
        std::uint32_t manufactureId;
        std::uint32_t productId;
        std::string manufactureName;
        std::string productName;
        std::string firmwareVersion;
        std::string hardwareVersion;
        std::string serialNumber;
    };

    struct MotionRequest
    {
        rpos::core::Vector2f linearSpeed;       // the unit is: m/s
        float angularSpeed;                     // the unit is: rad/s

                                                // both in right hand system, the head direction of the robot is x axis
                                                // the y axis point at the left of the robot

		MotionRequest()
			: linearSpeed(0.0f, 0.0f)
			, angularSpeed(0.0f)
		{
			//
		}
		void reset()
		{
			linearSpeed = rpos::core::Vector2f(0.0f, 0.0f);
			angularSpeed = 0.0f;
		}

		const float& vx() const { return linearSpeed.x(); }
		float& vx() { return linearSpeed.x(); }

		const float& vy() const { return linearSpeed.y(); }
		float& vy() { return linearSpeed.y(); }

		const float& omega() const { return angularSpeed; }
		float& omega() { return angularSpeed; }
    };

    struct MovementEstimation
    {
        rpos::core::Vector2f positionDifference;    // the unit is: m
        float angularDifference;                    // the unit is: rad

        MovementEstimation()
            : positionDifference(0.f, 0.f)
            , angularDifference(0.f)
        {
        }

        void reset()
        {
            positionDifference = rpos::core::Vector2f(0.f, 0.f);
            angularDifference = 0.f;
        }
#if 0
        rpos::core::Vector2f linearSpeed;           // the unit is: m/s
        float angularSpeed;                         // the unit is: rad/s
        rpos::core::Vector2f linearAcc;             // the unit is: m/s^2
        float angularAcc;                           // the unit is: rad/s^2
#endif
    };

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

    struct BaseError
    {
    public:
        BaseError()
            : id(0), errorCode(0), level(BaseErrorLevelWarn)
            , component(BaseErrorComponentSystem), componentErrorCode(0)
        {
        }

        int id;
        std::uint32_t errorCode;
        BaseErrorLevel level;
        BaseErrorComponent component;
        std::uint16_t componentErrorCode;
        std::string message;

        bool operator==(const BaseError & that)
        {
            return equals_(that);
        }

        bool operator==(const BaseError & that) const
        {
            return equals_(that);
        }

        bool operator!=(const BaseError & that)
        {
            return !equals_(that);
        }

        bool operator!=(const BaseError & that) const
        {
            return !equals_(that);
        }

    private:
        bool equals_(const BaseError & that) const
        {
            if (this == &that)
                return true;

            return this->errorCode == that.errorCode
                && this->level == that.level
                && this->component == that.component
                && this->componentErrorCode == that.componentErrorCode
                && this->message == that.message;
        }
    };

    struct BaseHealthInfo
    {
    public:
        BaseHealthInfo()
            : hasWarning(false), hasError(false), hasFatal(false)
        {}

        bool hasWarning;
        bool hasError;
        bool hasFatal;

        std::vector<BaseError> errors;

        bool operator==(const BaseHealthInfo & that)
        {
            return equals_(that);
        }

        bool operator==(const BaseHealthInfo & that) const
        {
            return equals_(that);
        }

        bool operator!=(const BaseHealthInfo & that)
        {
            return !equals_(that);
        }

        bool operator!=(const BaseHealthInfo & that) const
        {
            return !equals_(that);
        }

    private:
        bool equals_(const BaseHealthInfo & that) const
        {
            if (this == &that)
                return true;

            if (this->hasWarning != that.hasWarning
                || this->hasError != that.hasError
                || this->hasFatal != that.hasFatal)
                return false;

            for (auto iter = this->errors.cbegin(); iter != this->errors.cend(); ++iter)
            {
                if (!contains_(that, *iter))
                    return false;
            }

            for (auto iter = that.errors.cbegin(); iter != that.errors.cend(); ++iter)
            {
                if (!contains_(*this, *iter))
                    return false;
            }

            return true;
        }

        bool contains_(const BaseHealthInfo & info, const BaseError & error) const
        {
            for (auto iter = info.errors.cbegin(); iter != info.errors.cend(); ++iter)
            {
                if (*iter == error)
                    return true;
            }
            return false;
        }
    };

    struct AuxiliaryAnchorInfo
    {
        AuxiliaryAnchorInfo() {
            id = 0;
            distance_mm = 0;
            max_error_mm = 0;
        };
        uint16_t id;
        uint16_t distance_mm;
        uint8_t max_error_mm; // OPTIONAL, only for devices support error measurement
    };

    enum AuxiliaryAnchorType {
        AuxiliaryAnchorTypeUWB = 0,
    };

    struct AuxiliaryAnchor
    {
        AuxiliaryAnchor() {
            type = AuxiliaryAnchorTypeUWB;
            anchors_data.clear();
        };
        AuxiliaryAnchorType type;
        std::vector<AuxiliaryAnchorInfo> anchors_data;
    };


} } }
