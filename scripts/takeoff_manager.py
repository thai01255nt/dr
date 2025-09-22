#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

class TakeoffManager:
    def __init__(self):
        rospy.init_node('takeoff_manager')
        
        # Parameters
        self.required_altitude = rospy.get_param('~required_altitude', 1.0)  # 1m
        self.altitude_tolerance = rospy.get_param('~tolerance', 0.2)
        
        # Subscribers
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', 
                                        PoseStamped, self.pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        # State variables
        self.current_pose = PoseStamped()
        self.current_state = State()
        self.altitude_reached = False
        self.ready_announced = False
        
        # Check altitude every second
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self.check_altitude)
        
        rospy.loginfo(f"Takeoff Manager started. Monitoring altitude >= {self.required_altitude}m")
        rospy.loginfo("📋 Instructions:")
        rospy.loginfo("1. Use QGroundControl to takeoff drone")
        rospy.loginfo(f"2. Climb to at least {self.required_altitude}m altitude") 
        rospy.loginfo("3. Navigation system will be ready automatically")
        rospy.loginfo("4. Use RViz to set navigation goals")
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def check_altitude(self, event):
        current_alt = self.current_pose.pose.position.z
        
        # Check if altitude requirement met
        if current_alt >= (self.required_altitude - self.altitude_tolerance):
            if not self.altitude_reached:
                rospy.loginfo(f"✅ Altitude reached: {current_alt:.2f}m >= {self.required_altitude}m")
                self.altitude_reached = True
                
                # Set parameter để các node khác detect
                rospy.set_param('/takeoff_manager/altitude_reached', True)
                rospy.set_param('/takeoff_manager/stable_altitude', current_alt)
                
                # Announce system ready (once)
                if not self.ready_announced:
                    rospy.sleep(2.0)  # Wait a bit for stability
                    rospy.loginfo("🚁 ALTITUDE STABLE - STARTING NAVIGATION SYSTEM!")
                    rospy.loginfo("   • Cartographer: Starting map creation...")
                    rospy.loginfo("   • Move Base: Will start after map ready") 
                    rospy.loginfo("   • RViz: Will start after move_base ready")
                    rospy.loginfo("   • Goal Hover Manager: Active")
                    rospy.loginfo("")
                    rospy.loginfo("⏳ Please wait for all nodes to initialize...")
                    self.ready_announced = True
        else:
            if self.altitude_reached:  # Altitude dropped
                rospy.logwarn(f"⚠️  Altitude dropped: {current_alt:.2f}m < {self.required_altitude}m")
                self.altitude_reached = False
                # Clear parameter
                rospy.set_param('/takeoff_manager/altitude_reached', False)
                
        # Log current status every 5 seconds
        if int(time.time()) % 5 == 0:
            status_icon = "✅" if self.altitude_reached else "⏳"
            rospy.loginfo(f"{status_icon} Alt: {current_alt:.2f}m | "
                         f"Required: {self.required_altitude}m | "
                         f"Mode: {self.current_state.mode} | "
                         f"Ready: {self.altitude_reached}")

if __name__ == '__main__':
    try:
        manager = TakeoffManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Takeoff Manager shutting down")
