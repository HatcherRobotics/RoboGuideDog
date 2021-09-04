cmake_minimum_required(VERSION 2.8.3)
project(slamware_sdk)

find_package(catkin REQUIRED)

find_path(slamware_sdk_INCLUDE_DIR rpos/rpos.h ${PROJECT_SOURCE_DIR}/include)
find_path(slamware_sdk_LIBRARY librpos_framework.a ${PROJECT_SOURCE_DIR}/lib/linux-aarch64-gcc5.4)
if(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
    set(slamware_sdk_FOUND true)
else(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
    set(slamware_sdk_FOUND false)
endif(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
