/**
* feature.h
* The System Resource feature
*
* Created By Jacky Li @ 2014-7-1
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <map>

#include <rpos/rpos_config.h>
#include <rpos/core/feature.h>
#include <rpos/core/pose.h>
#include <rpos/core/parameter.h>

#include "system_parameters.h"
#include "laser_scan.h"
#include "device_info.h"
#include "device_health.h"
#include "power_management.h"
#include "../../message/depth_camera_messages.h"

namespace Json
{
    class Value;
}

namespace rpos { namespace features {

    namespace system_resource {

        enum RestartMode
        {
            RestartModeSoft,
            RestartModeHard
        };
        
        enum NetworkMode {
            NetworkModeAP,
            NetworkModeStation,
            NetworkModeWifiDisabled,
            NetworkModeDHCPDisabled,
            NetworkModeDHCPEnabled
        };

        struct NetworkStatus
        {
            std::map<std::string, std::string> options;
        };

        enum OperationAuditLevel
        {
            OperationAuditLevelCritical,
            OperationAuditLevelMajor,
            OperationAuditLevelMinor,
            OperationAuditLevelTrivial
        };

        struct OperationAuditLog
        {
            std::uint64_t timestamp;
            std::string peer_ip;
            OperationAuditLevel level;
            std::string source;
            std::string content;
            std::string timestampstr;
        };

        /// Represents the result of LIDAR auto tweak requests
        enum LidarAutoTweakRequestResult {
            /// The calibration process has been triggered and waiting for context
            LidarAutoTweakRequestResultStarted,

            /// The device or LIDAR doesn't support auto tweaking
            LidarAutoTweakRequestResultNotSupport = -1,

            /// The LIDAR auto tweaking service is not enabled in the config
            LidarAutoTweakRequestResultDisabled = -2
        };

        /// Represents the status of auto tweaking process
        enum LidarAutoTweakStatus {
            /// The tweaking process hasn't been triggered
            LidarAutoTweakStatusIdle,

            /// The tweaking process is started, but waiting for the availability of context
            LidarAutoTweakStatusWaitingForContext,

            /// The context is ready, the algorithm is doing the tweaking
            LidarAutoTweakStatusTweaking,

            /// The tweaking process has been finished, and applied on the fly.
            /// Notice: You still need invoke `acceptLidarAutoTweakResult()` to save it to LIDAR
            LidarAutoTweakStatusDone,

            /// The tweaking process is failed
            LidarAutoTweakStatusFailed = -1
        };

        typedef std::uint32_t HeartBeatToken;
        const std::uint32_t MAX_CONCURRENT_HEART_BEAT = 1;
    }

    namespace detail {
        class SystemResourceImpl;
    }

    class RPOS_CORE_API SystemResource : public rpos::core::Feature{
    public:
        typedef detail::SystemResourceImpl impl_t;

        RPOS_OBJECT_CTORS_WITH_BASE(SystemResource, rpos::core::Feature);
        SystemResource(boost::shared_ptr<detail::SystemResourceImpl> impl);
        ~SystemResource();

    public:
        int getBatteryPercentage();
        bool getBatteryIsCharging();
        bool getDCIsConnected();
        system_resource::PowerStatus getPowerStatus();
        void wakeUp();

        int getBoardTemperature();
        std::string getSDPVersion();
        system_resource::LaserScan getLaserScan();
        bool restartModule(system_resource::RestartMode mode = system_resource::RestartModeSoft);
        bool setSystemParameter(const std::string& param, const std::string& value);
        std::string getSystemParameter(const std::string& param);
        bool updateBinaryConfig(const Json::Value& jsnCfg);
        bool shutdownSlamcore(const rpos::core::SlamcoreShutdownParam& shutdownArg);
        system_resource::DeviceInfo getDeviceInfo();
        rpos::features::system_resource::BaseHealthInfo getRobotHealth();
        void clearRobotHealth(int errorCode);
        bool configurateNetwork(rpos::features::system_resource::NetworkMode mode, const std::map<std::string, std::string>& options);
        std::map<std::string, std::string> getNetworkStatus();
        system_resource::HeartBeatToken startHeartBeat(int heartBeatTimeoutInSeconds);
        void refreshHeartBeat(system_resource::HeartBeatToken token);
        void stopHeartBeat(system_resource::HeartBeatToken token);
        void voiceRespond();
        void startFirmwareUpgrade(const std::string& filename);
        void startFirmwareUpgrade(const std::vector<uint8_t>& firmwareContent);
        void publishDepthCamFrame(int sensorId, const rpos::message::depth_camera::DepthCameraFrame& frame, boost::optional<std::map<int, rpos::message::depth_camera::DepthCameraTransformParameters>> inputDepthCameraParams = boost::none);
        std::vector<system_resource::OperationAuditLog> getOperationAuditLogs();
        int sendAndRecvUserDefinedCBUSMessage(const void * payload, const size_t payloadsize, std::vector<std::uint8_t> & recvDat);

        ///////////////////////////////////////
        // LIDAR auto tweak relative methods //
        ///////////////////////////////////////

        /// Start the LIDAR auto tweaking process
        /// See definition of `LidarAutoTweakRequestResult` for details
        system_resource::LidarAutoTweakRequestResult beginLidarAutoTweak();

        /// Check current status of LIDAR auto tweaking process
        /// See definition of `LidarAutoTweakStatus` for details
        system_resource::LidarAutoTweakStatus getLidarAutoTweakStatus();

        /// Save LIDAR tweak result to LIDAR
        /// @return `true` for this operation has been done successfully, `false` for the operation failed in some reason (usually because of failure of auto-tweaking process).
        bool acceptLidarTweakResult();

        /// Reset LIDAR tweak result to its original values
        void resetLidarTweakResult();

        /// Abort the LIDAR auto tweaking process
        void cancelLidarAutoTweak();
    };

} }
