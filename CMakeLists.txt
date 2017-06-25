cmake_minimum_required(VERSION 2.8.3)
project(stanley)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  nav_msgs
  roscpp
  roslib
  rospy
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_kdl
  tf2_msgs
  tf2_py
  tf2_ros
  tf2_sensor_msgs
  tf2_tools
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pureStanley src/pureStanley.cpp)

target_link_libraries(pureStanley
   ${catkin_LIBRARIES}

 )
 
 add_executable(pureStanley2 src/pureStanley2.cpp)

target_link_libraries(pureStanley2
   ${catkin_LIBRARIES}

 )
