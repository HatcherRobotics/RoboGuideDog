
#include <slam_planner/slam_planner.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "move_base_node");
  
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  move_base::MoveBase planner( buffer );

  ros::spin();
  return(0);
}
