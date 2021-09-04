cmake_minimum_required(VERSION 2.8.3)
project(slam_planner_sdk)

find_package(catkin REQUIRED)

find_path(slam_planner_INCLUDE_DIR nav/MoveBaseConfig.h ${PROJECT_SOURCE_DIR}/include)
find_path(slam_planner_LIBRARY liblaikago_comm.so ${PROJECT_SOURCE_DIR}/lib)
find_path(laikago_sdk_INCLUDE_DIR laikago_sdk/comm.hpp ${PROJECT_SOURCE_DIR}/include)


if(slam_planner_INCLUDE_DIR AND slam_planner_LIBRARY)
    set(slam_planner_sdk_FOUND true)
else(slam_planner_INCLUDE_DIR AND slam_planner_LIBRARY)
    set(slam_planner_sdk_FOUND false)
endif(slam_planner_INCLUDE_DIR AND slam_planner_LIBRARY)

if(laikago_sdk_INCLUDE_DIR)
    set(laikago_sdk_INCLUDE_DIR true)
else(laikago_sdk_INCLUDE_DIR)
    set(laikago_sdk_INCLUDE_DIR false)
endif(laikago_sdk_INCLUDE_DIR)
