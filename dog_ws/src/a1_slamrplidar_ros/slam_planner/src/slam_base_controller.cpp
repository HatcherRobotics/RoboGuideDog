/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <slam_planner/HighCmd.h>
#include <slam_planner/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <slam_planner/convert.h>


using namespace UNITREE_LEGGED_SDK;

//static long motiontime = 0;
HighCmd SendHighLCM = {0};
HighState RecvHighLCM = {0};
slam_planner::HighCmd SendHighROS;
slam_planner::HighState RecvHighROS;

//Control control(LeggedType::A1, HIGHLEVEL);
LCM roslcm;
boost::mutex mutex;

void* update_loop(void* param)
{
    LCM *data = (LCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

void control_callback(const geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO("Linear Components:[%f,%f,%f]", cmd_vel.linear.x,  cmd_vel.linear.y,  cmd_vel.linear.z);
	ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
	
	SendHighROS.forwardSpeed = cmd_vel.linear.x;
	SendHighROS.rotateSpeed = cmd_vel.angular.z;
}

int main(int argc, char *argv[])
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl;
         //     << "Make sure the robot is standing on the ground." << std::endl
          //    << "Press Enter to continue..." << std::endl;
    //std::cin.ignore();

    ros::init(argc, argv, "base_controller_node");
    ros::NodeHandle n;
	
	SendHighROS.forwardSpeed = 0.0f;
	SendHighROS.sideSpeed = 0.0f;
	SendHighROS.rotateSpeed = 0.0f;
	SendHighROS.forwardSpeed = 0.0f;

	SendHighROS.mode = 0;
	SendHighROS.roll  = 0;
	SendHighROS.pitch = 0;
	SendHighROS.yaw = 0;

    ros::Subscriber sub = n.subscribe("/cmd_vel", 1, control_callback);

    ros::Rate loop_rate(500);
    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, &roslcm);

	while (ros::ok())
	{
		roslcm.Get(RecvHighLCM); // receive the A1's message.
		//memcpy(&RecvHighROS, &RecvHighLCM, sizeof(HighState));
		//printf("%f\n",  RecvHighROS.forwardSpeed);	

        // Just give a simple example: receive the velocity.
		if(SendHighROS.forwardSpeed != 0 || SendHighROS.rotateSpeed !=0)
		{
			SendHighROS.mode = 2;
		}
		else
		{
			SendHighROS.mode =1;
		}
        //memcpy(&SendHighLCM, &SendHighROS, sizeof(HighCmd));
		SendHighLCM = ToLcm(SendHighROS);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}
