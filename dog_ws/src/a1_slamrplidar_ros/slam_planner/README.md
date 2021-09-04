# slamrplidar

  A SLAM and Path PLanning system based a single rplidar.

 `[IMPORTANCE]`

 `Another information about the hardware connected, please refer ../rplidar_基于激光雷达的SLAM系统及路径规划功能使用手册2.0.PDF`

## 1 Running (Local Login)

1. First, start the laikago, waiting to communication:
    `$ cd ~/laikago_sdk/build`
    `$ sudo ./sdk_lcm_server_high`

2. Open a new terminal, start the rplidar and mapping:
    `$ roslaunch slam_planner slam_rplidar_start.launch`

3. In another terminal, launch the Path planning algorithm:
    `$ roslaunch slam_planner slam_planner_online.launch`

4. In another terminal, load the base_controller_node node to control laikago(*Auto: planning...*):
    `$ rosrun slam_planner base_controller_node`

## 2 Running (Remote Login: For TX2, it's not work profectly in some scenarios)

`This part for remote login test, example "ssh"`

1. First, login in the laikago's computer (ssh your_laikago's_ip_address), open two terminal:

    < Notice: Keep your computer with laikago's computer in a LAN. >

    `do 1.1, 1.2`

2. Then, open a new terminal: (ssh -X your_laikago's_ip_address)

    `do 1.3`

3. In another terminal, (ssh your_laikago's_ip_address):

    `do 1.4`

After complete above steps, the 2D-Map will be shown in the rviz.

And, you can use the `2D Nav Goal` button(or publish a message to `/move_base_simple/goal` topic ) to set the goal position and orientation.

The laikago will search the optimal path to reach the target point, with avoiding the dynamic obstacles.




