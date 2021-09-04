/**
* slamware_agent_platform.h
* Slamtec Slamware(r) Agent platform
*
* Created By Gabriel He @ 2016-01-28
* Copyright (c) 2016 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/features/artifact_provider.h>
#include <rpos/features/location_provider.h>
#include <rpos/features/motion_planner.h>
#include <rpos/features/sweep_motion_planner.h>
#include <rpos/features/system_resource.h>
#include <rpos/features/impact_sensor_feature.h>
#include <rpos/features/statistics_provider.h>
#include <rpos/features/sweep_motion_planner/sweep_region.h>
#include <rpos/robot_platforms/objects/composite_map.h>
#include <rpos/robot_platforms/objects/slamware_scheduler_service.h>
#include <rpos/robot_platforms/objects/slamware_firmware_service.h>
#include <rpos/robot_platforms/slamware_http_exception.h>
#include <rpos/core/parameter.h>
#include <boost/shared_ptr.hpp>

#include <vector>

namespace rpos { namespace robot_platforms {


    namespace detail {
        class SlamwareAgentPlatformImpl;
        class SlamwareActionFactory;
        class SlamwareHttpsClient;
    }

    using namespace robot_platforms::http;

    class RPOS_SLAMWARE_API SlamwareAgentPlatform
    {
        friend class detail::SlamwareActionFactory;
    public:
        typedef detail::SlamwareAgentPlatformImpl impl_t;

        SlamwareAgentPlatform();
        SlamwareAgentPlatform(boost::shared_ptr<impl_t> impl);
        ~SlamwareAgentPlatform();

    public:
        static SlamwareAgentPlatform connect(const std::string& host, int port);
        void disconnect();

    public:
        features::ArtifactProvider getArtifactProvider();
        features::LocationProvider getLocationProvider();
        features::MotionPlanner getMotionPlanner();
        features::SweepMotionPlanner getSweepMotionPlanner();
        features::SystemResource getSystemResource();
        features::ImpactSensor getImpactSensor();
        features::StatisticsProvider getStatisticsProvider();

    public:
        // Artifacts Provider APIs
        std::vector<core::Line> getLines(features::artifact_provider::ArtifactUsage usage);

        bool addLine(features::artifact_provider::ArtifactUsage usage, const core::Line& line);

        bool addLines(features::artifact_provider::ArtifactUsage usage, const std::vector<core::Line>& lines);

        bool removeLineById(features::artifact_provider::ArtifactUsage usage, rpos::core::SegmentID id);

        bool clearLines(features::artifact_provider::ArtifactUsage usage);

        bool moveLine(features::artifact_provider::ArtifactUsage usage, const core::Line& line);
        bool moveLines(features::artifact_provider::ArtifactUsage usage, const std::vector<core::Line>& lines);

        std::vector<core::Line> getWalls();

        bool addWall(const core::Line& wall);

        bool addWalls(const std::vector<core::Line>& walls);

        bool clearWallById(const core::SegmentID& id);

        bool clearWalls();

    public:
        // Location Provider APIs
        std::vector<features::location_provider::MapType> getAvailableMaps();

        features::location_provider::Map getMap(features::location_provider::MapType type, core::RectangleF area, features::location_provider::MapKind kind);

        bool setMapAndPose(const core::Pose& pose, const features::location_provider::Map& map, const features::location_provider::MapType& type, const features::location_provider::MapKind& kind, bool partially = false);
        bool setMap(const features::location_provider::Map& map, features::location_provider::MapType type, features::location_provider::MapKind kind, bool partially = false);

        core::RectangleF getKnownArea(features::location_provider::MapType type, features::location_provider::MapKind kind);

        bool clearMap();

        bool clearMap(features::location_provider::MapKind kind);

        core::Location getLocation();

        core::Pose getPose();

        bool setPose(const core::Pose& pose);

        bool getMapLocalization();

        bool setMapLocalization(bool localization);

        bool getMapUpdate(rpos::features::location_provider::MapKind kind = rpos::features::location_provider::EXPLORERMAP);

        bool setMapUpdate(bool update, rpos::features::location_provider::MapKind kind = rpos::features::location_provider::EXPLORERMAP);

        int getLocalizationQuality();

        features::location_provider::PointPDF getAuxLocation();

        bool getHomePose(core::Pose&);
        bool setHomePose(core::Pose);

    public:
        // Motion Planner APIs
        actions::MoveAction moveTo(const std::vector<core::Location>& locations, bool appending, bool isMilestone);

        actions::MoveAction moveTo(const core::Location& location, bool appending, bool isMilestone);

        actions::MoveAction moveTo(const std::vector<core::Location>& locations, const features::motion_planner::MoveOptions& options, float yaw = 0);

        actions::MoveAction moveTo(const core::Location& location, const features::motion_planner::MoveOptions& options, float yaw = 0);

        actions::MoveAction moveBy(const core::Direction& direction);

        actions::MoveAction moveBy(const core::Direction& direction, const features::motion_planner::MoveOptions& options);

        actions::MoveAction moveBy(float theta, const features::motion_planner::MoveOptions& options);

        actions::MoveAction rotateTo(const core::Rotation& orientation);

        actions::MoveAction rotateTo(const core::Rotation& orientation, const features::motion_planner::MoveOptions& options);

        actions::MoveAction rotate(const core::Rotation& rotation);

        actions::MoveAction rotate(const core::Rotation& rotation, const features::motion_planner::MoveOptions& options);

        actions::MoveAction recoverLocalization(const core::RectangleF& area, const features::motion_planner::RecoverLocalizationOptions& options=features::motion_planner::RecoverLocalizationOptions());
        
        actions::MoveAction recoverLocalizationByDock();

        rpos::actions::VelocityControlMoveAction velocityControl();

        actions::MoveAction getCurrentAction();

        std::vector<core::Location> getKeyPoints();

        void setKeyPoints(const std::vector<core::Location>& keyPoints);

        features::motion_planner::Path searchPath(const core::Location& location);

        features::motion_planner::Path getRobotTrack(int count);

        actions::SweepMoveAction startSweep();

        actions::SweepMoveAction sweepSpot(const core::Location& location);

        actions::MoveAction goHome();

        float getSweepArea();

        rpos::actions::SweepMoveAction startRegionSweep(const std::vector<size_t>& ids, const std::vector<size_t>& numbers);

        void insertRegion(const rpos::features::Region& region);

        void insertRegions(const std::vector<rpos::features::Region>& regions);

        void removeRegion(size_t id);

        void removeRegions(const std::vector<size_t>& ids);

        void updateRegion(const rpos::features::Region& region);

        void updateRegions(const std::vector<rpos::features::Region>& regions);

        std::vector<rpos::features::Region> getRegions();

        std::vector<rpos::features::Region> getSweepingRegions();

        rpos::actions::SweepMoveAction startFollowPathSweep(const rpos::features::motion_planner::Path& path);

        rpos::features::motion_planner::Path getPaintedSweepPath();

    public:
        // System Resource APIs
        int getBatteryPercentage();

        bool getBatteryIsCharging();

        bool getDCIsConnected();

        features::system_resource::PowerStatus getPowerStatus();

        void wakeUp();

        int getBoardTemperature();

        std::string getSDPVersion();

        std::string getSDKVersion();

        features::system_resource::LaserScan getLaserScan();

        bool restartModule(features::system_resource::RestartMode mode = features::system_resource::RestartModeSoft);

        bool setSystemParameter(const std::string& param, const std::string& value);

        std::string getSystemParameter(const std::string& param);

        bool updateBinaryConfig(const Json::Value& jsnCfg);

        bool shutdownSlamcore(const rpos::core::SlamcoreShutdownParam& shutdownArg);

        features::system_resource::DeviceInfo getDeviceInfo();


        features::system_resource::BaseHealthInfo getRobotHealth();
        void clearRobotHealth(int errorCode);

        bool configurateNetwork(features::system_resource::NetworkMode mode, const std::map<std::string, std::string>& options);

        std::map<std::string, std::string> getNetworkStatus();

        features::system_resource::HeartBeatToken startHeartBeat(int heartBeatTimeoutInSeconds);

        void refreshHeartBeat(features::system_resource::HeartBeatToken token);

        void stopHeartBeat(features::system_resource::HeartBeatToken token);

        void voiceRespond();

        std::vector<features::system_resource::OperationAuditLog> getOperationAuditLogs();

        int sendAndRecvUserDefinedCBUSMessage(const void * payload, const size_t payloadsize, std::vector<std::uint8_t> & recvDat);

    public:
        // Impact Sensor APIs
        bool getSensors(std::vector<features::impact_sensor::ImpactSensorInfo>& sensors);

        bool getSensorValues(std::map<features::impact_sensor::impact_sensor_id_t, features::impact_sensor::ImpactSensorValue>& values);

        bool getSensorValues(const std::vector<features::impact_sensor::impact_sensor_id_t>& sensorIds, std::vector<features::impact_sensor::ImpactSensorValue>& values);

        bool getSensorValue(features::impact_sensor::impact_sensor_id_t sensorId, features::impact_sensor::ImpactSensorValue& value);

        robot_platforms::objects::CompositeMap getCompositeMap();

        void setCompositeMap(const robot_platforms::objects::CompositeMap& map, const core::Pose& pose);


    public:
        // Scheduler Service APIs
        std::vector<detail::objects::ScheduledTask> getScheduledTasks();

        bool addScheduledTask(const detail::objects::ScheduledTask& task);

        detail::objects::ScheduledTask getScheduledTask(int taskId);

        detail::objects::ScheduledTask updateScheduledTask(const detail::objects::ScheduledTask& task);

        bool deleteScheduledTask(int taskId);

    public:
        // Firmware Service APIs
        detail::objects::UpdateInfo getUpdateInfo();

        bool startFirmwareUpdate();

        detail::objects::UpdateProgress getFirmwareUpdateProgress();

    private:
        boost::shared_ptr<detail::SlamwareHttpsClient> getHttpsClient();

    private:
        boost::shared_ptr<impl_t> impl_;
    };
}}
