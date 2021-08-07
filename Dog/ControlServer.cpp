/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <librealsense2/rs.hpp>

using namespace UNITREE_LEGGED_SDK;

HighCmd udpCommand;
int isCmdSetFinished = 0;
int isStopped = 1;
extern rs2::pipeline p;

LoopFunc* loop_control;
LoopFunc* loop_udpSend;
LoopFunc* loop_udpRecv;

class Custom
{
public:
    Custom(uint8_t level) : safe(LeggedType::A1), udp(level) {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = { 0 };
    HighState state = { 0 };
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

Custom custom(HIGHLEVEL);

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

void Custom::RobotControl()
{
    motiontime += 2;
    udp.GetRecv(state);
    // printf("%d   %f\n", motiontime, state.forwardSpeed);

    if (isStopped)
    {
        cmd.forwardSpeed = 0.0f;
        cmd.sideSpeed = 0.0f;
        cmd.rotateSpeed = 0.0f;
        cmd.bodyHeight = 0.0f;

        cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
        cmd.roll = 0;
        cmd.pitch = 0;
        cmd.yaw = 0;
    }

    if (isCmdSetFinished)
    {
        cmd = udpCommand;
        isCmdSetFinished = 0;
    }

    // safity check
    rs2::frameset frames = p.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();
    int height = depth.get_height();
    int width = depth.get_width();
    auto center_depth = 0.0f;
    for (int x = -10; x < 10; ++x)
        for (int y = -10; y < 10; ++y)
            center_depth += depth.get_distance(width / 2 + x, height / 2 + y);
    center_depth /= 400;

    if (center_depth < 1)
    {
        if (cmd.mode == 2 && cmd.forwardSpeed > 0)
        {
            cmd.mode = 1;
            cmd.forwardSpeed = 0;
            std::cout << "Too close: " << center_depth << "m" << std::endl;
        }
    }

    udp.SetSend(cmd);
}

void init_loop_server()
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
        << "WARNING: Make sure the robot is standing on the ground." << std::endl
        << "Press Enter to continue..." << std::endl;

    loop_control = new LoopFunc("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    loop_udpSend = new LoopFunc("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    loop_udpRecv = new LoopFunc("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend->start();
    loop_udpRecv->start();
    loop_control->start();
}
