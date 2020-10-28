cmake_minimum_required(VERSION 2.8.3)
project(pioneer_ufba)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  urdf
)

find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p3dx_description
#  CATKIN_DEPENDS roscpp rospy urdf
#  DEPENDS system_lib
  DEPENDS 
    roscpp 
    gazebo_ros 
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_library(set_speed_pioneer_plugin plugin/set_speed_pioneer_plugin.cpp)
target_link_libraries(set_speed_pioneer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
