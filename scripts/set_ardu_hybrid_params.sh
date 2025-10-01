#!/bin/bash
rosrun mavros mavparam load ardu_hybrid.param
rosservice call /mavros/param/push
rosservice call /mavros/cmd/command "command: 246
param1: 1.0
param2: 0.0
param3: 0.0
param4: 0.0
param5: 0.0
param6: 0.0
param7: 0.0"
