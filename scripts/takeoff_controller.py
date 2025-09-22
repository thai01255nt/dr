#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBoolRequest, CommandTOL, SetMode, CommandBool
from sensor_msgs.msg import NavSatFix
import threading

class TakeoffController:
    def __init__(self):
        rospy.init_node('takeoff_controller', anonymous=True)
        
        # Publishers
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        
        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.extended_state_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extended_state_cb)
        
        # Service clients
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        
        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.extended_state = ExtendedState()
        self.target_altitude = 1.0  # 1m target altitude
        self.position_tolerance = 0.1  # 10cm tolerance
        
        # Control flags
        self.takeoff_complete = False
        self.offboard_enabled = False
        
        rospy.loginfo("[Takeoff Controller] TakeoffController initialized")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def extended_state_cb(self, msg):
        self.extended_state = msg

    def wait_for_connection(self, timeout=30):
        """Wait for MAVROS connection"""
        rospy.loginfo("[Takeoff Controller] Waiting for MAVROS connection...")
        rate = rospy.Rate(20)
        start_time = time.time()
        
        while not rospy.is_shutdown() and not self.current_state.connected:
            if time.time() - start_time > timeout:
                rospy.logerr("[Takeoff Controller] Timeout waiting for MAVROS connection")
                return False
            rate.sleep()
        
        rospy.loginfo("[Takeoff Controller] MAVROS connected")
        return True

    def send_position_setpoint(self, x=0, y=0, z=1.0):
        """Send position setpoint"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # Keep current orientation
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        
        self.local_pos_pub.publish(pose)

    def set_offboard_mode(self):
        """Set vehicle to OFFBOARD mode"""
        if not self.current_state.connected:
            rospy.logwarn("[Takeoff Controller] Vehicle not connected")
            return False
        if self.current_state.mode != "OFFBOARD":
            try:
                resp = self.set_mode_client(0, "OFFBOARD")
                if resp.mode_sent:
                    rospy.loginfo("[Takeoff Controller] OFFBOARD mode set")
                    return True
                else:
                    rospy.logwarn("[Takeoff Controller] Failed to set OFFBOARD mode")
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(f"[Takeoff Controller] Service call failed: {e}")
                return False
        return True


    def arm_vehicle(self):
        """Arm the vehicle"""
        if not self.current_state.armed:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True
            
            if self.arming_client.call(arm_cmd).success:
                rospy.loginfo("[Takeoff Controller] Vehicle armed")
                return True
            else:
                rospy.logwarn("[Takeoff Controller] Failed to arm vehicle")
                return False
        return True

    def check_altitude_reached(self):
        """Check if target altitude is reached"""
        current_alt = self.current_pose.pose.position.z
        return abs(current_alt - self.target_altitude) < self.position_tolerance

    def takeoff_sequence(self):
        """Execute complete takeoff sequence to 1m altitude"""
        rospy.loginfo("[Takeoff Controller] Starting takeoff sequence...")
        
        # Wait for connection
        if not self.wait_for_connection():
            return False
        
        # Send a few setpoints before starting offboard mode
        rospy.loginfo("[Takeoff Controller] Sending initial setpoints...")
        rate = rospy.Rate(20)
        for i in range(100):
            if rospy.is_shutdown():
                return False
            self.send_position_setpoint(0, 0, self.target_altitude)
            rate.sleep()
        
        # Set OFFBOARD mode
        rospy.loginfo("[Takeoff Controller] Setting OFFBOARD mode...")
        if not self.set_offboard_mode():
            return False
        
        # Wait for mode to be set
        timeout = time.time() + 5
        while self.current_state.mode != "OFFBOARD" and time.time() < timeout:
            rate.sleep()
        
        if self.current_state.mode != "OFFBOARD":
            rospy.logerr("[Takeoff Controller] Failed to enter OFFBOARD mode")
            return False
        
        # Arm vehicle
        rospy.loginfo("[Takeoff Controller] Arming vehicle...")
        if not self.arm_vehicle():
            return False
        
        # Wait for arming
        timeout = time.time() + 5
        while not self.current_state.armed and time.time() < timeout:
            rate.sleep()
        
        if not self.current_state.armed:
            rospy.logerr("[Takeoff Controller] Failed to arm vehicle")
            return False
        
        # Takeoff to target altitude
        rospy.loginfo(f"[Takeoff Controller] Taking off to {self.target_altitude}m altitude...")
        start_time = time.time()
        takeoff_timeout = 30  # 30 seconds timeout
        
        while not rospy.is_shutdown():
            # Continue sending setpoints
            self.send_position_setpoint(0, 0, self.target_altitude)
            
            # Check if altitude reached
            if self.check_altitude_reached():
                rospy.loginfo(f"[Takeoff Controller] Target altitude {self.target_altitude}m reached!")
                self.takeoff_complete = True
                self.offboard_enabled = True
                
                # Hold position for 2 seconds to stabilize
                rospy.loginfo("[Takeoff Controller] Stabilizing at target altitude...")
                stabilize_start = time.time()
                while time.time() - stabilize_start < 2.0:
                    self.send_position_setpoint(0, 0, self.target_altitude)
                    rate.sleep()
                
                return True
            
            # Check timeout
            if time.time() - start_time > takeoff_timeout:
                rospy.logerr("[Takeoff Controller] Takeoff timeout!")
                return False
            
            # Check if still armed and in offboard
            if not self.current_state.armed:
                rospy.logerr("[Takeoff Controller] Vehicle disarmed during takeoff!")
                return False
            
            if self.current_state.mode != "OFFBOARD":
                rospy.logerr("[Takeoff Controller] Lost OFFBOARD mode during takeoff!")
                return False
            
            rate.sleep()
        
        return False

    def maintain_altitude(self):
        """Maintain current altitude and position"""
        if not self.takeoff_complete:
            rospy.logwarn("[Takeoff Controller] Takeoff not complete, cannot maintain altitude")
            return
        
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        self.send_position_setpoint(current_x, current_y, self.target_altitude)

    def emergency_land(self):
        """Emergency landing procedure"""
        rospy.logwarn("[Takeoff Controller] Initiating emergency landing...")
        
        land_mode = SetMode()
        land_mode.custom_mode = 'LAND'
        
        if self.set_mode_client.call(land_mode).mode_sent:
            rospy.loginfo("[Takeoff Controller] LAND mode set")
            self.takeoff_complete = False
            self.offboard_enabled = False
        else:
            rospy.logerr("[Takeoff Controller] Failed to set LAND mode")

def main():
    try:
        controller = TakeoffController()
        
        rospy.loginfo("[Takeoff Controller] Starting takeoff sequence...")
        if controller.takeoff_sequence():
            rospy.loginfo("[Takeoff Controller] Takeoff completed successfully!")
            
            # Keep the node alive and maintain position
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                controller.maintain_altitude()
                rate.sleep()
        else:
            rospy.logerr("[Takeoff Controller] Takeoff failed!")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("[Takeoff Controller] Takeoff controller interrupted")
    except Exception as e:
        rospy.logerr(f"Takeoff controller error: {e}")

if __name__ == '__main__':
    main()
