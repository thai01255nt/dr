#!/bin/bash
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
