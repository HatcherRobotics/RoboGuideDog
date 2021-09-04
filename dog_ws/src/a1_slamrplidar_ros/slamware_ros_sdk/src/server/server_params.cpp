
#include "server_params.h"

#include <cmath>

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    const float C_FLT_PI = ((float)M_PI);
    const float C_FLT_2PI = (C_FLT_PI * 2);

    //////////////////////////////////////////////////////////////////////////

    ServerParams::ServerParams()
    {
        resetToDefault();
    }

    void ServerParams::resetToDefault()
    {
        ip_address = "192.168.11.1";
        robot_port = 1445;
        reconn_wait_ms = (1000U * 3U);

        angle_compensate = true;
        fixed_odom_map_tf = true;

        robot_frame = "/base_link";
        laser_frame = "/laser";
        map_frame = "/map";
        odom_frame = "/odom";

        robot_pose_pub_period = 0.05f;
        scan_pub_period = 0.1f;
        map_update_period = 0.2f;
        map_pub_period = 0.2f;
        basic_sensors_info_update_period = 7.0f;
        basic_sensors_values_pub_period = 0.05f;
        path_pub_period = 0.05f;
        robot_basic_state_pub_period = 1.0f;
        virtual_walls_pub_period = 0.5f;
        virtual_tracks_pub_period = 0.5f;

        map_sync_once_get_max_wh = 100.f;
        map_update_near_robot_half_wh = 8.0f;

        scan_topic = "scan";
        odom_topic = "odom";
        map_topic = "map";
        map_info_topic = "map_metadata";
        basic_sensors_info_topic = "basic_sensors_info";
        basic_sensors_values_topic = "basic_sensors_values";
        path_topic = "global_plan_path";

        vel_control_topic = "/cmd_vel";
        goal_topic = "/move_base_simple/goaldd";
    }

    void ServerParams::setBy(const ros::NodeHandle& nhRos)
    {
        nhRos.getParam("ip_address", ip_address);
        nhRos.getParam("robot_port", robot_port);
        nhRos.getParam("reconn_wait_ms", reconn_wait_ms);

        nhRos.getParam("angle_compensate", angle_compensate);
        nhRos.getParam("fixed_odom_map_tf", fixed_odom_map_tf);

        nhRos.getParam("robot_frame", robot_frame);
        nhRos.getParam("laser_frame", laser_frame);
        nhRos.getParam("map_frame", map_frame);
        nhRos.getParam("odom_frame", odom_frame);

        nhRos.getParam("robot_pose_pub_period", robot_pose_pub_period);
        nhRos.getParam("scan_pub_period", scan_pub_period);
        nhRos.getParam("map_update_period", map_update_period);
        nhRos.getParam("map_pub_period", map_pub_period);
        nhRos.getParam("basic_sensors_info_update_period", basic_sensors_info_update_period);
        nhRos.getParam("basic_sensors_values_pub_period", basic_sensors_values_pub_period);
        nhRos.getParam("path_pub_period", path_pub_period);
        nhRos.getParam("robot_basic_state_pub_period", robot_basic_state_pub_period);
        nhRos.getParam("virtual_walls_pub_period", virtual_walls_pub_period);
        nhRos.getParam("virtual_tracks_pub_period", virtual_tracks_pub_period);

        nhRos.getParam("map_sync_once_get_max_wh", map_sync_once_get_max_wh);
        nhRos.getParam("map_update_near_robot_half_wh", map_update_near_robot_half_wh);

        nhRos.getParam("scan_topic", scan_topic);
        nhRos.getParam("odom_topic", odom_topic);
        nhRos.getParam("map_topic", map_topic);
        nhRos.getParam("map_info_topic", map_info_topic);
        nhRos.getParam("basic_sensors_info_topic", basic_sensors_info_topic);
        nhRos.getParam("basic_sensors_values_topic", basic_sensors_values_topic);
        nhRos.getParam("path_topic", path_topic);

        nhRos.getParam("vel_control_topic", vel_control_topic);
        nhRos.getParam("goal_topic", goal_topic);
    }

    //////////////////////////////////////////////////////////////////////////
    
}
