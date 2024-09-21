
cmake_minimum_required(VERSION 3.0.2)
project(jajusung_main)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)  
find_package(PCL REQUIRED)  
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  message_generation
  erp42_msgs
  image_transport
  pcl_ros
  pcl_conversions
)


add_message_files(
  FILES
  # ERP42CtrlCmd.msg
  lane_info.msg
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
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_follower src/lane_follower.cpp src/EKF_lane.cpp)
add_dependencies(lane_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_follower 
  ${catkin_LIBRARIES}
)

add_executable(gps_follower src/gps_stanley_main.cpp src/gps_stanley.cpp)
add_dependencies(gps_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_follower 
  ${catkin_LIBRARIES}
)

add_executable(fusion_node src/sensor_fusion.cpp)
target_link_libraries(fusion_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
set_property(TARGET fusion_node PROPERTY CXX_STANDARD 14)

find_package(OpenMP) #omp 설정 안쓰면 주석달기
if (OPENMP_FOUND)
  target_link_libraries(fusion_node OpenMP::OpenMP_CXX)
endif()