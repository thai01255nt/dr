# Autonomous Drone Navigation System

This ROS package implements autonomous drone navigation using Cartographer SLAM and TEB Local Planner.

## System Architecture

### Core Components
1. **Cartographer SLAM** - Real-time mapping and localization
2. **TEB Local Planner** - Dynamic obstacle avoidance and path planning  
3. **MAVROS/PX4** - Drone control interface
4. **Custom Python Controllers** - Intelligent flight management

## Quick Start

### Prerequisites
```bash
# Make sure PX4 and MAVROS are running first
roslaunch px4 px4.launch
roslaunch mavros px4.launch
```

### Run Autonomous Flight
```bash
# Start the complete autonomous flight sequence
rosrun cart_teb_test autonomous_flight_manager.py
```

This will automatically:
1. ✈️ Takeoff to 1m altitude
2. 🗺️ Start Cartographer mapping
3. 🧭 Launch navigation stack when map is ready
4. 🎯 Enable goal setting via RViz

## Python Scripts

### 1. `autonomous_flight_manager.py` (Main Controller)
**Purpose**: Orchestrates the complete flight sequence
**Features**:
- Automated takeoff to 1m altitude
- Sequential launch of Cartographer → move_base → RViz
- System health monitoring
- Graceful shutdown handling

**Usage**:
```bash
rosrun cart_teb_test autonomous_flight_manager.py
```

### 2. `takeoff_controller.py` (Takeoff Utility)
**Purpose**: Handles precise takeoff to 1m altitude
**Features**:
- Works from any flight mode (LOITER, STABILIZE, etc.)
- Automatic OFFBOARD mode activation
- Precise altitude control (±10cm tolerance)
- Safety timeouts and error handling

**Usage**:
```bash
rosrun cart_teb_test takeoff_controller.py
```

### 3. `teb_hover_controller.py` (Navigation Manager)
**Purpose**: Manages TEB planner and hover behavior
**Features**:
- Auto-detects when goals are reached
- Maintains OFFBOARD mode throughout flight
- Intelligent hover control at goal positions
- Monitors TEB planner activity and timeouts

**Topics**:
- Publishes: `/teb_controller/status`, `/teb_controller/goal_reached`
- Subscribes: `/move_base/status`, `/move_base/goal`, TEB cmd_vel

### 4. `emergency_controller.py` (Safety System)
**Purpose**: Emergency procedures and safety monitoring
**Features**:
- Multiple emergency modes (LAND, HOVER, RTL, STOP)
- Automatic safety triggers (altitude bounds, connection loss)
- Launch position recording for RTL
- Continuous safety monitoring

**Services**:
```bash
# Emergency landing
rosservice call /emergency_controller/emergency_land

# Emergency hover at current position  
rosservice call /emergency_controller/emergency_hover

# Return to launch position
rosservice call /emergency_controller/return_to_launch

# Immediate stop and hover
rosservice call /emergency_controller/emergency_stop

# Reset emergency state
rosservice call /emergency_controller/reset
```

## Flight Workflow

### Automated Sequence
1. **Initialization**: System checks and connections
2. **Takeoff**: Precise ascent to 1m altitude in OFFBOARD mode
3. **Mapping**: Cartographer starts building map from LIDAR+IMU+GPS
4. **Navigation Ready**: move_base and RViz launch when map is available
5. **Goal Navigation**: Set goals in RViz → TEB plans path → Auto hover when reached

### Goal Setting Process
1. 🎯 **Set Goal**: Use RViz "2D Nav Goal" tool
2. 🚁 **TEB Control**: Local planner takes control and navigates
3. ⏸️ **Auto Hover**: When goal reached, automatically hover at position
4. 🔄 **Repeat**: Set new goal to continue navigation

## Best Practices

### Safety Features
- **Continuous OFFBOARD**: Never switches flight modes during navigation
- **Failsafe Monitoring**: Connection loss, altitude bounds checking
- **Emergency Services**: Instant emergency procedures available
- **Position Holding**: Maintains precise hover between goals

### Operational Tips
- Always run PX4/MAVROS first before starting autonomous flight
- Monitor `/flight_manager/status` topic for system state
- Use emergency services if needed: `rosservice call /emergency_controller/emergency_land`
- System automatically maintains altitude and position safety

## Configuration

### TEB Planner Settings
- Max velocity: 1.5 m/s (x,y), 1.0 rad/s (yaw)
- Goal tolerance: 0.2m position, 0.1 rad orientation
- Circular footprint: 0.3m radius
- Holonomic robot configuration (can strafe)

### Cartographer Settings  
- 2D SLAM with LIDAR + IMU + GPS fusion
- Range: 0.1-30m, 5cm resolution
- Map frame optimization every 35 nodes

## Troubleshooting

### Common Issues
- **Takeoff fails**: Check MAVROS connection and PX4 mode
- **No map**: Verify LIDAR topics (`/scan`) are publishing
- **Navigation fails**: Ensure map is published (`/map` topic)
- **Emergency needed**: Use emergency services immediately

### Status Topics
- `/flight_manager/status` - Overall system status
- `/teb_controller/status` - Navigation controller status  
- `/emergency_controller/status` - Safety system status

## Dependencies
- ROS Noetic
- PX4 + MAVROS
- Cartographer ROS
- move_base + TEB local planner
- Python 3.x
