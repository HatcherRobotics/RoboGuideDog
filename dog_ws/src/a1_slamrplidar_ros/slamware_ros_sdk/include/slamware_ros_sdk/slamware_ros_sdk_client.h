
#pragma once

#include <ros/ros.h>
#include <tf/message_filter.h>

#include "utils.h"

#include <slamware_ros_sdk/Vec2DInt32.h>
#include <slamware_ros_sdk/Line2DFlt32Array.h>
#include <slamware_ros_sdk/RectInt32.h>
#include <slamware_ros_sdk/RobotDeviceInfo.h>
#include <slamware_ros_sdk/BasicSensorInfoArray.h>
#include <slamware_ros_sdk/BasicSensorValueDataArray.h>
#include <slamware_ros_sdk/RobotBasicState.h>
#include <slamware_ros_sdk/SyncMapRequest.h>
#include <slamware_ros_sdk/MoveByDirectionRequest.h>
#include <slamware_ros_sdk/MoveByThetaRequest.h>
#include <slamware_ros_sdk/MoveToRequest.h>
#include <slamware_ros_sdk/MoveToLocationsRequest.h>
#include <slamware_ros_sdk/RotateToRequest.h>
#include <slamware_ros_sdk/RotateRequest.h>
#include <slamware_ros_sdk/RecoverLocalizationRequest.h>
#include <slamware_ros_sdk/ClearMapRequest.h>
#include <slamware_ros_sdk/SetMapUpdateRequest.h>
#include <slamware_ros_sdk/SetMapLocalizationRequest.h>
#include <slamware_ros_sdk/GoHomeRequest.h>
#include <slamware_ros_sdk/CancelActionRequest.h>
#include <slamware_ros_sdk/AddLineRequest.h>
#include <slamware_ros_sdk/AddLinesRequest.h>
#include <slamware_ros_sdk/RemoveLineRequest.h>
#include <slamware_ros_sdk/ClearLinesRequest.h>
#include <slamware_ros_sdk/MoveLineRequest.h>
#include <slamware_ros_sdk/MoveLinesRequest.h>

#include <slamware_ros_sdk/SyncGetStcm.h>
#include <slamware_ros_sdk/SyncSetStcm.h>

#include <boost/filesystem/path.hpp>

namespace slamware_ros_sdk {

    class SlamwareRosSdkClient
    {
    public:
        typedef boost::filesystem::path             fs_path_t;

    public:
        explicit SlamwareRosSdkClient(ros::NodeHandle& nhRos
            , const char* serverNodeName = nullptr
            , const char* msgNamePrefix = nullptr
            );
        ~SlamwareRosSdkClient();

    public:
        //////////////////////////////////////////////////////////////////////////

        void syncMap(const SyncMapRequest& msg) { return pubSyncMap_.publish(msg); }
        void setPose(const geometry_msgs::Pose& msg) { pubSetPose_.publish(msg); }

        void recoverLocalization(const RecoverLocalizationRequest& msg) { pubRecoverLocalization_.publish(msg); }
        void clearMap(const ClearMapRequest& msg) { pubClearMap_.publish(msg); }
        void setMapUpdate(const SetMapUpdateRequest& msg) { pubSetMapUpdate_.publish(msg); }
        void setMapLocalization(const SetMapLocalizationRequest& msg) { pubSetMapLocalization_.publish(msg); }

        void moveBy(const MoveByDirectionRequest& msg) { pubMoveByDirection_.publish(msg); }
        void moveBy(const MoveByThetaRequest& msg) { pubMoveByTheta_.publish(msg); }
        void moveTo(const MoveToRequest& msg) { pubMoveTo_.publish(msg); }
        void moveTo(const MoveToLocationsRequest& msg) { pubMoveToLocations_.publish(msg); }
        void rotateTo(const RotateToRequest& msg) { pubRotateTo_.publish(msg); }
        void rotate(const RotateRequest& msg) { pubRotate_.publish(msg); }

        void goHome(const GoHomeRequest& msg) { pubGoHome_.publish(msg); }
        void cancelAction(const CancelActionRequest& msg) { pubCancelAction_.publish(msg); }

        void addLine(const AddLineRequest& msg) { pubAddLine_.publish(msg); }
        void addLines(const AddLinesRequest& msg) { pubAddLines_.publish(msg); }
        void removeLine(const RemoveLineRequest& msg) { pubRemoveLine_.publish(msg); }
        void clearLines(const ClearLinesRequest& msg) { pubClearLines_.publish(msg); }
        void moveLine(const MoveLineRequest& msg) { pubMoveLine_.publish(msg); }
        void moveLines(const MoveLinesRequest& msg) { pubMoveLines_.publish(msg); }

        //////////////////////////////////////////////////////////////////////////

        bool syncGetStcm(SyncGetStcm& srvMsg) { return scSyncGetStcm_.call(srvMsg); }
        // get stcm and write to filePath.
        bool syncGetStcm(std::string& errMsg
            , const fs_path_t& filePath
            );

        bool syncSetStcm(SyncSetStcm& srvMsg) { return scSyncSetStcm_.call(srvMsg); }
        // load stcm from filePath, and upload to slamware.
        bool syncSetStcm(const fs_path_t& filePath
            , const geometry_msgs::Pose& robotPose
            , std::string& errMsg
            );

        //////////////////////////////////////////////////////////////////////////

    private:
        std::string genTopicFullName_(const std::string& strName) const { return msgNamePrefix_ + strName; }

    private:
        ros::NodeHandle* nh_;
        std::string sdkServerNodeName_;
        std::string msgNamePrefix_;

        ros::Publisher pubSyncMap_;
        ros::Publisher pubSetPose_;

        ros::Publisher pubRecoverLocalization_;
        ros::Publisher pubClearMap_;
        ros::Publisher pubSetMapUpdate_;
        ros::Publisher pubSetMapLocalization_;

        ros::Publisher pubMoveByDirection_;
        ros::Publisher pubMoveByTheta_;
        ros::Publisher pubMoveTo_;
        ros::Publisher pubMoveToLocations_;
        ros::Publisher pubRotateTo_;
        ros::Publisher pubRotate_;

        ros::Publisher pubGoHome_;
        ros::Publisher pubCancelAction_;

        ros::Publisher pubAddLine_;
        ros::Publisher pubAddLines_;
        ros::Publisher pubRemoveLine_;
        ros::Publisher pubClearLines_;
        ros::Publisher pubMoveLine_;
        ros::Publisher pubMoveLines_;

        ros::ServiceClient scSyncGetStcm_;
        ros::ServiceClient scSyncSetStcm_;
    };

}
