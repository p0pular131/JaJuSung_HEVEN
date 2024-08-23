
cmake_minimum_required(VERSION 3.0.2)
project(jajusung_main)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  erp42_msgs
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  # ERP42CtrlCmd.msg
  # lane_info.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES erp42_msgs jajusung_main
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(lane_follower src/lane_follower.cpp src/EKF_lane.cpp)

add_dependencies(lane_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(lane_follower 
  ${catkin_LIBRARIES}
)
