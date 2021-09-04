
#pragma once

#include <ros/ros.h>

#include "server_map_holder.h"
#include "msg_convert.h"

#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>

namespace slamware_ros_sdk {

    struct ServerWorkData
    {
    public:
        typedef rpos::features::impact_sensor::ImpactSensorInfo                 sensor_info_t;
        typedef rpos::features::impact_sensor::ImpactSensorValue                sensor_value_t;
    	typedef std::map<int, sensor_info_t>                                    sensors_info_map_t;
    	typedef std::map<int, sensor_value_t>                                   sensors_values_map_t;

        typedef slamware_ros_sdk::BasicSensorInfo                               ros_basic_sensor_info_t;
        typedef std::map<int, ros_basic_sensor_info_t>                          ros_basic_sensor_info_map_t;

    public:
        RobotDeviceInfo robotDeviceInfo;

        rpos::core::Pose robotPose;

        boost::atomic<bool> syncMapRequested;
        ServerMapHolder exploreMapHolder;

        sensors_info_map_t sensorsInfo;
        ros_basic_sensor_info_map_t rosBasicSensorsInfo;

    public:
        ServerWorkData();

    public:
        static inline bool sfIsDigitalSensorValueImpact(float fVal) { return fVal < FLT_EPSILON; }
    };

    typedef boost::shared_ptr<ServerWorkData>               ServerWorkData_Ptr;
    typedef boost::shared_ptr<const ServerWorkData>         ServerWorkData_ConstPtr;
    
}
