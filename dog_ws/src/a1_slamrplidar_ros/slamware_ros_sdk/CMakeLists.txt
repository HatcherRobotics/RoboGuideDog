cmake_minimum_required(VERSION 2.8.3)
project(slamware_ros_sdk)

find_path(slamware_sdk_INCLUDE_DIR rpos/rpos.h ${PROJECT_SOURCE_DIR}/../slamware_sdk/include)
find_path(slamware_sdk_LIBRARY librpos_framework.a ${PROJECT_SOURCE_DIR}/../slamware_sdk/lib/linux-aarch64-gcc5.4)
if(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  set(SLTC_SDK_INC_DIR "${slamware_sdk_INCLUDE_DIR}")
  set(SLTC_SDK_LIB_DIR "${slamware_sdk_LIBRARY}")
else(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  if(NOT DEFINED SLTC_SDK_ROOT OR SLTC_SDK_ROOT STREQUAL "")
    set(SLTC_SDK_ROOT "/home/slamware/mywd/slamtec/slamware_sdk_linux-x86_64-gcc5.4")
  endif()
  set(SLTC_SDK_DIR "${SLTC_SDK_ROOT}/linux-x86_64-release")
  set(SLTC_SDK_INC_DIR "${SLTC_SDK_DIR}/include")
  set(SLTC_SDK_LIB_DIR "${SLTC_SDK_DIR}/lib")
endif(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=gnu++11")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)

find_package(Threads)

add_message_files(
  FILES
  OptionalBool.msg
  OptionalInt8.msg
  OptionalInt16.msg
  OptionalInt32.msg
  OptionalInt64.msg
  OptionalUInt8.msg
  OptionalUInt16.msg
  OptionalUInt32.msg
  OptionalUInt64.msg
  OptionalFlt32.msg
  OptionalFlt64.msg
  Vec2DInt32.msg
  Vec2DFlt32.msg
  Line2DFlt32.msg
  Line2DFlt32Array.msg
  RectInt32.msg
  RectFlt32.msg
  RobotDeviceInfo.msg
  MapKind.msg
  ArtifactUsage.msg
  SensorType.msg
  ImpactType.msg
  BasicSensorInfo.msg
  BasicSensorInfoArray.msg
  BasicSensorValue.msg
  BasicSensorValueData.msg
  BasicSensorValueDataArray.msg
  ActionDirection.msg
  RobotBasicState.msg
  SyncMapRequest.msg
  MoveOptionFlag.msg
  MoveOptions.msg
  MoveByDirectionRequest.msg
  MoveByThetaRequest.msg
  MoveToRequest.msg
  MoveToLocationsRequest.msg
  RotateToRequest.msg
  RotateRequest.msg
  LocalizationMovement.msg
  OptionalLocalizationMovement.msg
  LocalizationOptions.msg
  RecoverLocalizationRequest.msg
  ClearMapRequest.msg
  SetMapUpdateRequest.msg
  SetMapLocalizationRequest.msg
  GoHomeRequest.msg
  CancelActionRequest.msg
  AddLineRequest.msg
  AddLinesRequest.msg
  RemoveLineRequest.msg
  ClearLinesRequest.msg
  MoveLineRequest.msg
  MoveLinesRequest.msg
)

add_service_files(
  FILES
  SyncGetStcm.srv
  SyncSetStcm.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES slamware_ros_sdk
  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs tf message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
message("${INCLUDE_DIRECTORIES}")

add_library(${PROJECT_NAME} STATIC
  src/client/slamware_ros_sdk_client.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(slamware_ros_sdk_server_node
  src/server/msg_convert.cpp
  src/server/server_params.cpp
  src/server/server_map_holder.cpp
  src/server/server_work_data.cpp
  src/server/server_worker_base.cpp
  src/server/server_workers.cpp
  src/server/slamware_ros_sdk_server_node.cpp
  src/server/slamware_ros_sdk_server.cpp
)
add_dependencies(slamware_ros_sdk_server_node ${PROJECT_NAME}_generate_messages_cpp)
target_include_directories(slamware_ros_sdk_server_node
  PRIVATE ${SLTC_SDK_INC_DIR}
)
target_compile_options(slamware_ros_sdk_server_node
  PRIVATE -Wno-deprecated-declarations
)
target_link_libraries(slamware_ros_sdk_server_node
  ${catkin_LIBRARIES}
  ${SLTC_SDK_LIB_DIR}/librpos_robotplatforms_rpslamware.a
  ${SLTC_SDK_LIB_DIR}/librpos_framework.a
  ${SLTC_SDK_LIB_DIR}/libbase64.a
  ${SLTC_SDK_LIB_DIR}/librlelib.a
  ${SLTC_SDK_LIB_DIR}/libjsoncpp.a
  ${SLTC_SDK_LIB_DIR}/libcurl.a
  ${SLTC_SDK_LIB_DIR}/libssl.a
  ${SLTC_SDK_LIB_DIR}/libcrypto.a
  ${SLTC_SDK_LIB_DIR}/libboost_atomic.a
  ${SLTC_SDK_LIB_DIR}/libboost_chrono.a
  ${SLTC_SDK_LIB_DIR}/libboost_date_time.a
  ${SLTC_SDK_LIB_DIR}/libboost_regex.a
  ${SLTC_SDK_LIB_DIR}/libboost_filesystem.a
  ${SLTC_SDK_LIB_DIR}/libboost_system.a
  ${SLTC_SDK_LIB_DIR}/libboost_thread.a
  ${SLTC_SDK_LIB_DIR}/libboost_random.a
  pthread
  dl
  rt
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  launch/slamware_ros_sdk_server_node.launch
  launch/view_slamware_ros_sdk_server_node.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
