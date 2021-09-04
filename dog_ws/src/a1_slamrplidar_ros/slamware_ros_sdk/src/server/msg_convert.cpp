
#include "msg_convert.h"

#include <stdexcept>

namespace slamware_ros_sdk {

	//////////////////////////////////////////////////////////////////////////

    void MsgConvert<MapKind, rpos::features::location_provider::MapKind>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        switch (sltcVal)
        {
        case rpos::features::location_provider::EXPLORERMAP:
            rosMsg.kind = ros_msg_t::EXPLORERMAP;
            break;
        case rpos::features::location_provider::SWEEPERMAP:
            rosMsg.kind = ros_msg_t::SWEEPERMAP;
            break;
        case rpos::features::location_provider::UWBMAP:
            rosMsg.kind = ros_msg_t::UWBMAP;
            break;
        case rpos::features::location_provider::SLAMMAP:
            rosMsg.kind = ros_msg_t::SLAMMAP;
            break;
        case rpos::features::location_provider::LOCALSLAMMAP:
            rosMsg.kind = ros_msg_t::LOCALSLAMMAP;
            break;
        default:
            rosMsg.kind = ros_msg_t::UNKNOWN;
            break;
        }
    }

    void MsgConvert<MapKind, rpos::features::location_provider::MapKind>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        switch (rosMsg.kind)
        {
        case ros_msg_t::EXPLORERMAP:
            sltcVal = rpos::features::location_provider::EXPLORERMAP;
            break;
        case ros_msg_t::SWEEPERMAP:
            sltcVal = rpos::features::location_provider::SWEEPERMAP;
            break;
        case ros_msg_t::UWBMAP:
            sltcVal = rpos::features::location_provider::UWBMAP;
            break;
        case ros_msg_t::SLAMMAP:
            sltcVal = rpos::features::location_provider::SLAMMAP;
            break;
        case ros_msg_t::LOCALSLAMMAP:
            sltcVal = rpos::features::location_provider::LOCALSLAMMAP;
            break;
        default:
            //sltcVal = ((rpos::features::location_provider::MapKind)-1);
            throw std::runtime_error("no MapKind::UNKNOWN in RPOS.");
            break;
        }
    }

    void MsgConvert<ArtifactUsage, rpos::features::artifact_provider::ArtifactUsage>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        switch (sltcVal)
        {
        case rpos::features::artifact_provider::ArtifactUsageVirtualWall:
            rosMsg.usage = ros_msg_t::VIRTUAL_WALL;
            break;
        case rpos::features::artifact_provider::ArtifactUsageVirtualTrack:
            rosMsg.usage = ros_msg_t::VIRTUAL_TRACK;
            break;
        default:
            rosMsg.usage = ros_msg_t::UNKNOWN;
            break;
        }
    }

    void MsgConvert<ArtifactUsage, rpos::features::artifact_provider::ArtifactUsage>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        switch (rosMsg.usage)
        {
        case ros_msg_t::VIRTUAL_WALL:
            sltcVal = rpos::features::artifact_provider::ArtifactUsageVirtualWall;
            break;
        case ros_msg_t::VIRTUAL_TRACK:
            sltcVal = rpos::features::artifact_provider::ArtifactUsageVirtualTrack;
            break;
        default:
            //sltcVal = ((rpos::features::artifact_provider::ArtifactUsage)-1);
            throw std::runtime_error("no ArtifactUsageUnknown in RPOS.");
            break;
        }
    }

    void MsgConvert<SensorType, rpos::core::SensorType>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
    	switch (sltcVal)
        {
        case rpos::core::SensorTypeBumper:
            rosMsg.type = ros_msg_t::BUMPER;
            break;
        case rpos::core::SensorTypeCliff:
            rosMsg.type = ros_msg_t::CLIFF;
            break;
        case rpos::core::SensorTypeSonar:
            rosMsg.type = ros_msg_t::SONAR;
            break;
        case rpos::core::SensorTypeDepthCamera:
            rosMsg.type = ros_msg_t::DEPTH_CAMERA;
            break;
        case rpos::core::SensorTypeWallSensor:
            rosMsg.type = ros_msg_t::WALL_SENSOR;
            break;
        case rpos::core::SensorTypeMagTapeDetector:
            rosMsg.type = ros_msg_t::MAG_TAPE_DETECTOR;
            break;
        default:
            rosMsg.type = ros_msg_t::UNKNOWN;
            break;
        }
    }

    void MsgConvert<SensorType, rpos::core::SensorType>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
    	switch (rosMsg.type)
    	{
    	case ros_msg_t::BUMPER:
    		sltcVal = rpos::core::SensorTypeBumper;
    		break;
    	case ros_msg_t::CLIFF:
    		sltcVal = rpos::core::SensorTypeCliff;
    		break;
    	case ros_msg_t::SONAR:
    		sltcVal = rpos::core::SensorTypeSonar;
    		break;
    	case ros_msg_t::DEPTH_CAMERA:
    		sltcVal = rpos::core::SensorTypeDepthCamera;
    		break;
    	case ros_msg_t::WALL_SENSOR:
    		sltcVal = rpos::core::SensorTypeWallSensor;
    		break;
    	case ros_msg_t::MAG_TAPE_DETECTOR:
    		sltcVal = rpos::core::SensorTypeMagTapeDetector;
    		break;
    	default:
    		sltcVal = rpos::core::SensorTypeUnknown;
    		break;
    	}
    }

    void MsgConvert<ImpactType, rpos::features::impact_sensor::ImpactSensorType>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
    	switch (sltcVal)
        {
        case rpos::features::impact_sensor::ImpactSensorTypeDigital:
            rosMsg.type = ros_msg_t::DIGITAL;
            break;
        case rpos::features::impact_sensor::ImpactSensorTypeAnalog:
            rosMsg.type = ros_msg_t::ANALOG;
            break;
        default:
            rosMsg.type = ros_msg_t::UNKNOWN;
            break;
        }
    }

    void MsgConvert<ImpactType, rpos::features::impact_sensor::ImpactSensorType>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
    	switch (rosMsg.type)
    	{
    	case ros_msg_t::DIGITAL:
    		sltcVal = rpos::features::impact_sensor::ImpactSensorTypeDigital;
    		break;
    	case ros_msg_t::ANALOG:
    		sltcVal = rpos::features::impact_sensor::ImpactSensorTypeAnalog;
    		break;
    	default:
    		//sltcVal = rpos::features::impact_sensor::ImpactSensorTypeUnknown;
            throw std::runtime_error("no ImpactSensorTypeUnknown in RPOS.");
    		break;
    	}
    }

    void MsgConvert<BasicSensorInfo, rpos::features::impact_sensor::ImpactSensorInfo>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.id = sltcVal.id;
        sltcToRosMsg(sltcVal.coreSensorType, rosMsg.sensor_type);
        sltcToRosMsg(sltcVal.type, rosMsg.impact_type);
        sltcToRosMsg(sltcVal.pose, rosMsg.install_pose);
        rosMsg.refresh_freq = sltcVal.refreshFreq;
    }

    void MsgConvert<BasicSensorInfo, rpos::features::impact_sensor::ImpactSensorInfo>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal.id = rosMsg.id;
        rosMsgToSltc(rosMsg.sensor_type, sltcVal.coreSensorType);
        rosMsgToSltc(rosMsg.impact_type, sltcVal.type);
        rosMsgToSltc(rosMsg.install_pose, sltcVal.pose);
        sltcVal.refreshFreq = rosMsg.refresh_freq;
    }

    void MsgConvert<ActionDirection, rpos::core::ACTION_DIRECTION>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
    	switch (sltcVal)
    	{
    	case rpos::core::FORWARD:
    		rosMsg.direction = ros_msg_t::FORWARD;
    		break;
    	case rpos::core::BACKWARD:
    		rosMsg.direction = ros_msg_t::BACKWARD;
    		break;
    	case rpos::core::TURNRIGHT:
    		rosMsg.direction = ros_msg_t::TURNRIGHT;
    		break;
    	case rpos::core::TURNLEFT:
    		rosMsg.direction = ros_msg_t::TURNLEFT;
    		break;
    	default:
    		rosMsg.direction = ros_msg_t::UNKNOWN;
    		break;
    	}
    }

    void MsgConvert<ActionDirection, rpos::core::ACTION_DIRECTION>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
    	switch (rosMsg.direction)
    	{
    	case ros_msg_t::FORWARD:
    		sltcVal = rpos::core::FORWARD;
    		break;
    	case ros_msg_t::BACKWARD:
    		sltcVal = rpos::core::BACKWARD;
    		break;
    	case ros_msg_t::TURNRIGHT:
    		sltcVal = rpos::core::TURNRIGHT;
    		break;
    	case ros_msg_t::TURNLEFT:
    		sltcVal = rpos::core::TURNLEFT;
    		break;
    	default:
    		//sltcVal = ((rpos::core::ACTION_DIRECTION)-1);
    		sltcVal = rpos::core::INVALIDDIRECTION;
    		break;
    	}
    }

    void MsgConvert<MoveOptionFlag, rpos::features::motion_planner::MoveOptionFlag>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
    	// Note: currently just use the SAME interger values.
    	rosMsg.flags = static_cast<std::uint32_t>(sltcVal);
    }

    void MsgConvert<MoveOptionFlag, rpos::features::motion_planner::MoveOptionFlag>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
    	// Note: currently just use the SAME interger values.
    	sltcVal = static_cast<rpos::features::motion_planner::MoveOptionFlag>(rosMsg.flags);
    }

    void MsgConvert<MoveOptions, rpos::features::motion_planner::MoveOptions>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
    	sltcToRosMsg(sltcVal.flag, rosMsg.opt_flags);
    	optionalToRosMsg(sltcVal.speed_ratio, rosMsg.speed_ratio);
    }

    void MsgConvert<MoveOptions, rpos::features::motion_planner::MoveOptions>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
    	rosMsgToSltc(rosMsg.opt_flags, sltcVal.flag);
    	rosMsgToOptional(rosMsg.speed_ratio, sltcVal.speed_ratio);
    }

    void MsgConvert<geometry_msgs::Point, rpos::core::Location>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
    	rosMsg.x = sltcVal.x();
    	rosMsg.y = sltcVal.y();
    	rosMsg.z = sltcVal.z();
    }

    void MsgConvert<geometry_msgs::Point, rpos::core::Location>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
    	sltcVal.x() = rosMsg.x;
    	sltcVal.y() = rosMsg.y;
    	sltcVal.z() = rosMsg.z;
    }

    void MsgConvert<geometry_msgs::Quaternion, rpos::core::Rotation>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg = tf::createQuaternionMsgFromRollPitchYaw(sltcVal.roll(), sltcVal.pitch(), sltcVal.yaw());
    }

    void MsgConvert<geometry_msgs::Quaternion, rpos::core::Rotation>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        if (0.0 == rosMsg.x && 0.0 == rosMsg.y && 0.0 == rosMsg.z && 0.0 == rosMsg.w)
        {
            sltcVal = sltc_val_t(0.0, 0.0, 0.0);
            return;
        }

        const tf::Quaternion tq(rosMsg.x, rosMsg.y, rosMsg.z, rosMsg.w);
        const tf::Matrix3x3 tMat(tq);
        tMat.getRPY(sltcVal.roll(), sltcVal.pitch(), sltcVal.yaw());

        if (std::isnan(sltcVal.roll()))
        {
            ROS_WARN("Quaternion to RPY, roll is nan, set to zero.");
            sltcVal.roll() = 0.0;
        }
        if (std::isnan(sltcVal.pitch()))
        {
            ROS_WARN("Quaternion to RPY, pitch is nan, set to zero.");
            sltcVal.pitch() = 0.0;
        }
        if (std::isnan(sltcVal.yaw()))
        {
            ROS_WARN("Quaternion to RPY, yaw is nan, set to zero.");
            sltcVal.yaw() = 0.0;
        }
    }

    void MsgConvert<geometry_msgs::Pose, rpos::core::Pose>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        sltcToRosMsg(sltcVal.location(), rosMsg.position);
        sltcToRosMsg(sltcVal.rotation(), rosMsg.orientation);
    }

    void MsgConvert<geometry_msgs::Pose, rpos::core::Pose>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        rosMsgToSltc(rosMsg.position, sltcVal.location());
        rosMsgToSltc(rosMsg.orientation, sltcVal.rotation());
    }

    void MsgConvert<Vec2DInt32, rpos::core::Vector2i>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.x = sltcVal.x();
        rosMsg.y = sltcVal.y();
    }

    void MsgConvert<Vec2DInt32, rpos::core::Vector2i>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal = sltc_val_t(rosMsg.x, rosMsg.y);
    }

    void MsgConvert<Vec2DFlt32, rpos::core::Vector2f>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.x = sltcVal.x();
        rosMsg.y = sltcVal.y();
    }

    void MsgConvert<Vec2DFlt32, rpos::core::Vector2f>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal = sltc_val_t(rosMsg.x, rosMsg.y);
    }

    void MsgConvert<Vec2DFlt32, rpos::core::Point>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.x = sltcVal.x();
        rosMsg.y = sltcVal.y();
    }

    void MsgConvert<Vec2DFlt32, rpos::core::Point>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal = sltc_val_t(rosMsg.x, rosMsg.y);
    }

    void MsgConvert<Line2DFlt32, rpos::core::Line>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.id = sltcVal.id();
        sltcToRosMsg(sltcVal.startP(), rosMsg.start);
        sltcToRosMsg(sltcVal.endP(), rosMsg.end);
    }

    void MsgConvert<Line2DFlt32, rpos::core::Line>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal.id() = rosMsg.id;
        rosMsgToSltc(rosMsg.start, sltcVal.startP());
        rosMsgToSltc(rosMsg.end, sltcVal.endP());
    }

    void MsgConvert<RectInt32, rpos::core::RectangleI>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.x = sltcVal.x();
        rosMsg.y = sltcVal.y();
        rosMsg.w = sltcVal.width();
        rosMsg.h = sltcVal.height();
    }

    void MsgConvert<RectInt32, rpos::core::RectangleI>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal = sltc_val_t(rosMsg.x, rosMsg.y, rosMsg.w, rosMsg.h);
    }

    void MsgConvert<RectFlt32, rpos::core::RectangleF>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.x = sltcVal.x();
        rosMsg.y = sltcVal.y();
        rosMsg.w = sltcVal.width();
        rosMsg.h = sltcVal.height();
    }

    void MsgConvert<RectFlt32, rpos::core::RectangleF>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal = sltc_val_t(rosMsg.x, rosMsg.y, rosMsg.w, rosMsg.h);
    }

    void MsgConvert<LocalizationMovement, rpos::features::motion_planner::RecoverLocalizationMovement>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        switch (sltcVal)
        {
        case rpos::features::motion_planner::RecoverLocalizationMovementNoMove:
            rosMsg.type = ros_msg_t::NO_MOVE;
            break;
        case rpos::features::motion_planner::RecoverLocalizationMovementRotateOnly:
            rosMsg.type = ros_msg_t::ROTATE_ONLY;
            break;
        case rpos::features::motion_planner::RecoverLocalizationMovementAny:
            rosMsg.type = ros_msg_t::ANY;
            break;
        default:
            rosMsg.type = ros_msg_t::UNKNOWN;
            break;
        }
    }

    void MsgConvert<LocalizationMovement, rpos::features::motion_planner::RecoverLocalizationMovement>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        switch (rosMsg.type)
        {
        case ros_msg_t::NO_MOVE:
            sltcVal = rpos::features::motion_planner::RecoverLocalizationMovementNoMove;
            break;
        case ros_msg_t::ROTATE_ONLY:
            sltcVal = rpos::features::motion_planner::RecoverLocalizationMovementRotateOnly;
            break;
        case ros_msg_t::ANY:
            sltcVal = rpos::features::motion_planner::RecoverLocalizationMovementAny;
            break;
        default:
            sltcVal = rpos::features::motion_planner::RecoverLocalizationMovementUnknown;
            break;
        }
    }

    void MsgConvert<LocalizationOptions, rpos::features::motion_planner::RecoverLocalizationOptions>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        optionalToRosMsg(sltcVal.maxRecoverTimeInMilliSeconds, rosMsg.max_time_ms);
        toRosOptionalMsg(sltcVal.recoverMovementType, rosMsg.mvmt_type);
    }

    void MsgConvert<LocalizationOptions, rpos::features::motion_planner::RecoverLocalizationOptions>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        rosMsgToOptional(rosMsg.max_time_ms, sltcVal.maxRecoverTimeInMilliSeconds);
        fromRosOptionalMsg(rosMsg.mvmt_type, sltcVal.recoverMovementType);
    }

    //////////////////////////////////////////////////////////////////////////
    
}
