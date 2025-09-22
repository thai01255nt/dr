#!/usr/bin/env python3

import rospy
import subprocess
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_srvs.srv import Empty

class TakeoffManager:
    def __init__(self):
        rospy.init_node('takeoff_manager')
        
        # Parameters
        self.required_altitude = rospy.get_param('~required_altitude', 0.8)  # X meters
        self.altitude_tolerance = rospy.get_param('~tolerance', 0.1)
        
        # Subscribers
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', 
                                        PoseStamped, self.pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        # State variables
        self.current_pose = PoseStamped()
        self.current_state = State()
        self.altitude_reached = False
        self.nodes_started = False
        
        # Check altitude every second
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self.check_altitude)
        
        rospy.loginfo(f"Takeoff Manager started. Waiting for altitude >= {self.required_altitude}m")
        rospy.loginfo("Please takeoff using QGroundControl first!")
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def check_altitude(self, event):
        current_alt = self.current_pose.pose.position.z
        
        # Check if altitude requirement met
        if current_alt >= (self.required_altitude - self.altitude_tolerance):
            if not self.altitude_reached:
                rospy.loginfo(f"Altitude reached: {current_alt:.2f}m >= {self.required_altitude}m")
                self.altitude_reached = True
                self.start_navigation_nodes()
        else:
            if self.altitude_reached:  # Altitude dropped
                rospy.logwarn(f"Altitude dropped: {current_alt:.2f}m < {self.required_altitude}m")
                self.altitude_reached = False
                
        # Log current status every 5 seconds
        if int(time.time()) % 5 == 0:
            rospy.loginfo(f"Current altitude: {current_alt:.2f}m, "
                         f"Required: {self.required_altitude}m, "
                         f"State: {self.current_state.mode}")
            
    def start_navigation_nodes(self):
        """Start Cartographer, TEB, and RViz nodes sequentially"""
        if self.nodes_started:
            return
            
        rospy.loginfo("Starting navigation nodes...")
        
        try:
            # Step 1: Start Cartographer
            rospy.loginfo("Starting Cartographer...")
            subprocess.Popen([
                'roslaunch', 'cart_teb_test', 'real_device.launch',
                'start_cartographer:=true'
            ])
            
            # Wait for Cartographer to initialize
            rospy.sleep(5.0)
            
            # Wait for map to be published
            self.wait_for_topic('/map', timeout=30.0)
            rospy.loginfo("Cartographer map ready!")
            
            # Step 2: Start Move Base
            rospy.loginfo("Starting Move Base...")
            subprocess.Popen([
                'roslaunch', 'cart_teb_test', 'real_device.launch',
                'start_move_base:=true'
            ])
            
            # Wait for move_base to initialize
            rospy.sleep(3.0)
            self.wait_for_topic('/move_base/status', timeout=15.0)
            rospy.loginfo("Move Base ready!")
            
            # Step 3: Start RViz
            rospy.loginfo("Starting RViz...")
            subprocess.Popen([
                'roslaunch', 'cart_teb_test', 'real_device.launch',
                'start_rviz:=true'
            ])
            
            self.nodes_started = True
            rospy.loginfo("All navigation nodes started successfully!")
            rospy.loginfo("You can now set goals in RViz!")
            
        except Exception as e:
            rospy.logerr(f"Failed to start nodes: {e}")
            
    def wait_for_topic(self, topic_name, timeout=10.0):
        """Wait for a topic to be published"""
        rospy.loginfo(f"Waiting for topic: {topic_name}")
        try:
            rospy.wait_for_message(topic_name, rospy.AnyMsg, timeout=timeout)
            return True
        except rospy.ROSException:
            rospy.logwarn(f"Timeout waiting for topic: {topic_name}")
            return False

if __name__ == '__main__':
    try:
        manager = TakeoffManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Takeoff Manager shutting down")

