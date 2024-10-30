
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
  image_transport
  pcl_ros
  pcl_conversions
)


add_message_files(
  FILES
  # ERP42CtrlCmd.msg
  lane_info.msg
  HevenCtrlCmd.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES jajusung_main
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

# add_executable(gps_follower src/gps_stanley_main.cpp src/gps_stanley.cpp)
# add_dependencies(gps_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} # #{catkin_EXPORTED_TARGETS})
#target_link_libraries(gps_follower 
 # ${catkin_LIBRARIES}
#)



add_executable(gps_follower src/gps_stanley_main_usingekf.cpp src/gps_stanley_usingekf.cpp)
add_dependencies(gps_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_follower 
  ${catkin_LIBRARIES}
)

add_executable(fusion_node fusion_main/jonghap_sensor_fusion.cpp)
target_link_libraries(fusion_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
set_property(TARGET fusion_node PROPERTY CXX_STANDARD 14)

add_executable(fusion_johyang fusion_main/johyang_sensor_fusion.cpp)
target_link_libraries(fusion_johyang
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
set_property(TARGET fusion_node PROPERTY CXX_STANDARD 14)

add_executable(fusion_stanley fusion_main/stanley_fusion.cpp)
target_link_libraries(fusion_stanley
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(jedong_stanley fusion_main/jedong_stanley.cpp)
target_link_libraries(jedong_stanley
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

