cmake_minimum_required(VERSION 3.0.2)
project(cart_teb_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_ros
  gazebo_plugins
  xacro
)

catkin_package()
catkin_install_python(PROGRAMS
  scripts/publish_init_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
