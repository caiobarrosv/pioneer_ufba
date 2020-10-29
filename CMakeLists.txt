cmake_minimum_required(VERSION 2.8.3)
project(pioneer_ufba)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
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

add_executable(pioneer_tf_broadcaster src/pioneer_tf_broadcaster.cpp)
target_link_libraries(pioneer_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(pioneer_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(pioneer_odom_publisher src/pioneer_odom_publisher.cpp)
target_link_libraries(pioneer_odom_publisher ${catkin_LIBRARIES})
add_dependencies(pioneer_odom_publisher ${catkin_EXPORTED_TARGETS})