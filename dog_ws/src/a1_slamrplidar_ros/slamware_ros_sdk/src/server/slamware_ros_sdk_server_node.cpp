/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721

  modified by yun.li@slamtec.com, 2019.
*/

#include <ros/ros.h>

#include  "slamware_ros_sdk_server.h"

int main(int argc, char** argv)
{
  std::string errMsg;
  ros::init(argc, argv, "slamware_ros_sdk_server_node");

  {
    slamware_ros_sdk::SlamwareRosSdkServer rosSdkServer;
    if (!rosSdkServer.startRun(errMsg))
    {
      ROS_ERROR("failed to start slamware ros sdk server: %s.", errMsg.c_str());
      return -1;
    }

    ros::spin();

    rosSdkServer.requestStop();
    rosSdkServer.waitUntilStopped();
  }
  return 0;
}
