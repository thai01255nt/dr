cmake_minimum_required(VERSION 3.0.2)
project(cart_teb_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  gazebo_ros
  gazebo_plugins
  xacro
  cv_bridge
  image_transport
  robot_localization
)

find_package(OpenCV REQUIRED)

# AirSim Library - adjust path if needed
set(AIRSIM_ROOT /home/thai/Developments/Cosys-AirSim)
if(EXISTS "${AIRSIM_ROOT}")
  message(STATUS "Found AirSim at: ${AIRSIM_ROOT}")
  include_directories(
    include
    ${AIRSIM_ROOT}/AirLib/deps/rpclib/include
    ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${AIRSIM_ROOT}/AirLib/include
  )
  link_directories(
    ${AIRSIM_ROOT}/AirLib/deps/MavLinkCom/lib
    ${AIRSIM_ROOT}/AirLib/deps/rpclib/lib
    ${AIRSIM_ROOT}/AirLib/lib
  )
else()
  message(WARNING "AirSim not found at ${AIRSIM_ROOT}. C++ node may not build.")
endif()

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs cv_bridge image_transport robot_localization
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# AirSim Camera Node (C++)
if(EXISTS "${AIRSIM_ROOT}")
  add_executable(airsim_camera_node
    src/airsim_camera_node.cpp
  )

  target_link_libraries(airsim_camera_node
    ${AIRSIM_ROOT}/AirLib/lib/libAirLib.a
    ${AIRSIM_ROOT}/AirLib/deps/rpclib/lib/librpc.a
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    pthread
  )

  add_dependencies(airsim_camera_node
    ${catkin_EXPORTED_TARGETS}
  )
endif()

# Python scripts
catkin_install_python(PROGRAMS
  scripts/takeoff_and_hover.py
  scripts/teb_controller.py
  scripts/simple_goal_forwarder.py
  scripts/takeoff_and_hover_ardupilot.py
  scripts/qr_code_publisher.py
  scripts/qr_code_subscriber.py
  scripts/airsim_lidar_sensor.py
  scripts/teb_to_mavros_velocity.py
  scripts/airsim_camera_sensor.py
  scripts/fake_clock.py
  scripts/airsim_imu_sensor.py
  scripts/odom_timeout_checker.py
  scripts/odom_forwarder.py
  scripts/pose_to_odom.py
  scripts/airsim_publish.py
  scripts/real_camera_publisher.py
  scripts/calibrate_camera_charuco.py
  scripts/real_camera_v4l2_direct.py
  scripts/real_camera_v4l2_minimal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
