
#pragma once

#include <ros/ros.h>
#include <tf/message_filter.h>

#include <slamware_ros_sdk/utils.h>

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

#include <rpos/core/geometry.h>
#include <rpos/features/artifact_provider.h>
#include <rpos/features/location_provider.h>
#include <rpos/features/motion_planner.h>
#include <rpos/features/system_resource.h>
#include <rpos/features/impact_sensor_feature.h>

#include <vector>
#include <map>

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////
    // Important Notes:
    //      Generally, MsgConvert just overwrites known fields;
    //          unknown fields, which are new added and their codes are
    //          not added into MsgConvert, will be unchanged.
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    
    template<class RosMsgT, class SltcValT>
    struct MsgConvert;

    template<class RosMsgT, class SltcValT>
    inline void sltcToRosMsg(const SltcValT& sltcVal, RosMsgT& rosMsg)
    {
        MsgConvert<RosMsgT, SltcValT>::toRos(sltcVal, rosMsg);
    }
    template<class RosMsgT, class SltcValT>
    inline void rosMsgToSltc(const RosMsgT& rosMsg, SltcValT& sltcVal)
    {
        MsgConvert<RosMsgT, SltcValT>::toSltc(rosMsg, sltcVal);
    }

    //////////////////////////////////////////////////////////////////////////

    template<class ValT, class RosMsgT>
    inline void toRosOptionalMsg(const boost::optional<ValT> & optVal, RosMsgT& rosMsg)
    {
        if (optVal)
        {
            rosMsg.is_valid = true;
            sltcToRosMsg(*optVal, rosMsg.value);
        }
        else
        {
            rosMsg = RosMsgT();
        }
    }

    template<class ValT, class RosMsgT>
    inline void fromRosOptionalMsg(const RosMsgT& rosMsg, boost::optional<ValT> & optVal)
    {
        if (rosMsg.is_valid)
        {
            optVal = ValT();
            rosMsgToSltc(rosMsg.value, *optVal);
        }
        else
        {
            optVal.reset();
        }
    }

    //////////////////////////////////////////////////////////////////////////

    template<>
    struct MsgConvert<MapKind, rpos::features::location_provider::MapKind>
    {
    public:
        typedef MapKind                                        ros_msg_t;
        typedef rpos::features::location_provider::MapKind     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<ArtifactUsage, rpos::features::artifact_provider::ArtifactUsage>
    {
    public:
        typedef ArtifactUsage                                         ros_msg_t;
        typedef rpos::features::artifact_provider::ArtifactUsage      sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<SensorType, rpos::core::SensorType>
    {
    public:
        typedef SensorType                  ros_msg_t;
        typedef rpos::core::SensorType      sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<ImpactType, rpos::features::impact_sensor::ImpactSensorType>
    {
    public:
        typedef ImpactType                                          ros_msg_t;
        typedef rpos::features::impact_sensor::ImpactSensorType     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<BasicSensorInfo, rpos::features::impact_sensor::ImpactSensorInfo>
    {
    public:
        typedef BasicSensorInfo                                     ros_msg_t;
        typedef rpos::features::impact_sensor::ImpactSensorInfo     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<ActionDirection, rpos::core::ACTION_DIRECTION>
    {
    public:
        typedef ActionDirection                     ros_msg_t;
        typedef rpos::core::ACTION_DIRECTION        sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<MoveOptionFlag, rpos::features::motion_planner::MoveOptionFlag>
    {
    public:
        typedef MoveOptionFlag                                  ros_msg_t;
        typedef rpos::features::motion_planner::MoveOptionFlag  sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<MoveOptions, rpos::features::motion_planner::MoveOptions>
    {
    public:
        typedef MoveOptions                                     ros_msg_t;
        typedef rpos::features::motion_planner::MoveOptions     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<geometry_msgs::Point, rpos::core::Location>
    {
    public:
        typedef geometry_msgs::Point                ros_msg_t;
        typedef rpos::core::Location                sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<geometry_msgs::Quaternion, rpos::core::Rotation>
    {
    public:
        typedef geometry_msgs::Quaternion           ros_msg_t;
        typedef rpos::core::Rotation                sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<geometry_msgs::Pose, rpos::core::Pose>
    {
    public:
        typedef geometry_msgs::Pose                 ros_msg_t;
        typedef rpos::core::Pose                    sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<Vec2DInt32, rpos::core::Vector2i>
    {
    public:
        typedef Vec2DInt32                        ros_msg_t;
        typedef rpos::core::Vector2i              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    // Vec2DFlt32 <<======>> rpos::core::Vector2f
    template<>
    struct MsgConvert<Vec2DFlt32, rpos::core::Vector2f>
    {
    public:
        typedef Vec2DFlt32                        ros_msg_t;
        typedef rpos::core::Vector2f              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };
    // Vec2DFlt32 <<======>> rpos::core::Point
    template<>
    struct MsgConvert<Vec2DFlt32, rpos::core::Point>
    {
    public:
        typedef Vec2DFlt32                        ros_msg_t;
        typedef rpos::core::Point                 sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<Line2DFlt32, rpos::core::Line>
    {
    public:
        typedef Line2DFlt32                       ros_msg_t;
        typedef rpos::core::Line                  sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<RectInt32, rpos::core::RectangleI>
    {
    public:
        typedef RectInt32                           ros_msg_t;
        typedef rpos::core::RectangleI              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<RectFlt32, rpos::core::RectangleF>
    {
    public:
        typedef RectFlt32                           ros_msg_t;
        typedef rpos::core::RectangleF              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<LocalizationMovement, rpos::features::motion_planner::RecoverLocalizationMovement>
    {
    public:
        typedef LocalizationMovement                                            ros_msg_t;
        typedef rpos::features::motion_planner::RecoverLocalizationMovement     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<LocalizationOptions, rpos::features::motion_planner::RecoverLocalizationOptions>
    {
    public:
        typedef LocalizationOptions                                            ros_msg_t;
        typedef rpos::features::motion_planner::RecoverLocalizationOptions     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    //////////////////////////////////////////////////////////////////////////

    template<class RosMsgT, class SltcValT, class RosVecAllocT, class SltcVecAllocT>
    struct MsgConvert< std::vector<RosMsgT, RosVecAllocT> , std::vector<SltcValT, SltcVecAllocT> >
    {
    public:
        typedef std::vector<RosMsgT, RosVecAllocT>              ros_msg_t;
        typedef std::vector<SltcValT, SltcVecAllocT>            sltc_val_t;

    public:
        static void toRos(const sltc_val_t& vSltcVal, ros_msg_t& vRosMsg)
        {
            const size_t szCnt = vSltcVal.size();
            vRosMsg.resize(szCnt);
            for (size_t t = 0; t < szCnt; ++t)
            {
                sltcToRosMsg(vSltcVal[t], vRosMsg[t]);
            }
        }

        static void toSltc(const ros_msg_t& vRosMsg, sltc_val_t& vSltcVal)
        {
            const size_t szCnt = vRosMsg.size();
            vSltcVal.resize(szCnt);
            for (size_t t = 0; t < szCnt; ++t)
            {
                rosMsgToSltc(vRosMsg[t], vSltcVal[t]);
            }
        }
    };

    template<class RosKeyT, class SltcKeyT, class RosMsgT, class SltcValT, class RosCmpT, class SltcCmpT, class RosAllocT, class SltcAllocT>
    struct MsgConvert< std::map<RosKeyT, RosMsgT, RosCmpT, RosAllocT>, std::map<SltcKeyT, SltcValT, SltcCmpT, SltcAllocT> >
    {
    public:
        typedef std::map<RosKeyT, RosMsgT, RosCmpT, RosAllocT>              ros_msg_t;
        typedef std::map<SltcKeyT, SltcValT, SltcCmpT, SltcAllocT>          sltc_val_t;

    public:
        static void toRos(const sltc_val_t& mapSltcVal, ros_msg_t& mapRosMsg)
        {
            mapRosMsg.clear();
            for (auto cit = mapSltcVal.cbegin(), citEnd = mapSltcVal.cend(); citEnd != cit; ++cit)
            {
                sltcToRosMsg(cit->second, mapRosMsg[cit->first]);
            }
        }

        static void toSltc(const ros_msg_t& mapRosMsg, sltc_val_t& mapSltcVal)
        {
            mapSltcVal.clear();
            for (auto cit = mapRosMsg.cbegin(), citEnd = mapRosMsg.cend(); citEnd != cit; ++cit)
            {
                rosMsgToSltc(cit->second, mapSltcVal[cit->first]);
            }
        }
    };

    //////////////////////////////////////////////////////////////////////////
    
}
