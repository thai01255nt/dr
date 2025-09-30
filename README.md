rosservice call /mavros/cmd/command "broadcast: false
command: 511 # MAV_CMD_SET_MESSAGE_INTERVAL
confirmation: 0
param1: 26 # message_id (ATTITUDE_QUATERNION)
param2: 100000 # interval_us (10Hz)
param3: 0.0
param4: 0.0
param5: 0.0
param6: 0.0
param7: 0.0"
rosservice call /mavros/cmd/command "broadcast: false
command: 511 # MAV_CMD_SET_MESSAGE_INTERVAL
confirmation: 0
param1: 31 # message_id (ATTITUDE_QUATERNION)
param2: 100000 # interval_us (10Hz)
param3: 0.0
param4: 0.0
param5: 0.0
param6: 0.0
param7: 0.0"

rosservice call /mavros/cmd/set_home "current_gps: true
latitude: 0.0
longitude: 0.0
altitude: 0.0"

source /opt/ros/noetic/setup.bash
source slam_ws/devel_isolated/setup.bash
source cart_teb_test/devel/setup.bash
export ROS_MASTER_URI=http://192.168.123.112:11311
export ROS_IP=192.168.123.112

source /opt/ros/noetic/setup.bash
source catkin_ws/devel_isolated/setup.bash
source cart_teb_test/devel/setup.bash
export ROS_MASTER_URI=http://192.168.123.129:11311
export ROS_IP=192.168.123.129

```
.~/cart_teb_test/src/cart_teb_test/scripts/set_ardu_gps_params.sh
.~/cart_teb_test/src/cart_teb_test/scripts/set_ardu_non_gps_params.sh

roslaunch cart_teb_test real-device.launch
roslaunch cart_teb_test mavros.launch fcu_url:="/dev/ttyUSB0"
roslaunch ldlidar_ros ld19.launch
# require imu
.~/cart_teb_test/src/cart_teb_test/scripts/set_ardu_publish_imu.sh
roslaunch cart_teb_test cartographer.launch use_gps:=false
rosrun cart_teb_test autonomous_flight_manager.py
```

# ARDUPILOT SIMULATOR

```
git clone https://github.com/ArduPilot/ardupilot.git --recursive
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y

cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

then edit model to add lidar + gps (or without gps)

```
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models
export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
```

first time to install sitl and gazebo classic

```
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
gazebo --verbose worlds/iris_arducopter_runway.world
```
