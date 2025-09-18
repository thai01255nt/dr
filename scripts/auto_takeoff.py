#!/usr/bin/env python3

import rospy
import time
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
import threading

class AutoTakeoff:
    def __init__(self):
        rospy.init_node('auto_takeoff_node', anonymous=True)
        
        # Publishers
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        
        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pos_callback)
        
        # Service clients
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_altitude = 2.0  # 2 meters
        self.takeoff_complete = False
        
        # Wait for connection
        self.rate = rospy.Rate(20)  # 20Hz
        
        rospy.loginfo("Auto Takeoff Node Started")
        rospy.loginfo("Waiting for MAVROS connection...")
        
        # Wait for MAVROS connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()
        
        rospy.loginfo("MAVROS connected!")
        
        # Start takeoff sequence
        self.start_takeoff_sequence()
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def local_pos_callback(self, msg):
        self.current_pose = msg
        
        # Check if takeoff is complete (reached target altitude)
        current_altitude = msg.pose.position.z
        if current_altitude >= (self.target_altitude - 0.2) and not self.takeoff_complete:
            self.takeoff_complete = True
            rospy.loginfo(f"✅ TAKEOFF COMPLETE! Current altitude: {current_altitude:.2f}m")
            rospy.loginfo("🚀 Ready to start navigation nodes...")
            
            # Create a flag file to signal other nodes can start
            with open('/tmp/drone_takeoff_complete', 'w') as f:
                f.write('READY')
                
    def wait_for_service(self, service_name, timeout=30):
        """Wait for a service to be available"""
        rospy.loginfo(f"Waiting for service: {service_name}")
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
            rospy.loginfo(f"✅ Service {service_name} is available")
            return True
        except rospy.ROSException:
            rospy.logerr(f"❌ Service {service_name} not available after {timeout}s")
            return False
            
    def set_mode(self, mode):
        """Set flight mode"""
        if not self.wait_for_service('/mavros/set_mode', 10):
            return False
            
        try:
            response = self.set_mode_client(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"✅ Mode set to: {mode}")
                return True
            else:
                rospy.logwarn(f"❌ Failed to set mode: {mode}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Service call failed: {e}")
            return False
            
    def arm_vehicle(self):
        """Arm the vehicle"""
        if not self.wait_for_service('/mavros/cmd/arming', 10):
            return False
            
        try:
            response = self.arming_client(value=True)
            if response.success:
                rospy.loginfo("✅ Vehicle ARMED")
                return True
            else:
                rospy.logwarn("❌ Failed to ARM vehicle")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Arming service call failed: {e}")
            return False
            
    def takeoff(self):
        """Initiate takeoff"""
        if not self.wait_for_service('/mavros/cmd/takeoff', 10):
            return False
            
        try:
            response = self.takeoff_client(
                altitude=self.target_altitude,
                latitude=0,
                longitude=0,
                min_pitch=0,
                yaw=0
            )
            if response.success:
                rospy.loginfo(f"✅ Takeoff command sent! Target altitude: {self.target_altitude}m")
                return True
            else:
                rospy.logwarn("❌ Takeoff command failed")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Takeoff service call failed: {e}")
            return False
            
    def start_takeoff_sequence(self):
        """Execute the complete takeoff sequence"""
        rospy.loginfo("🚁 Starting Automatic Takeoff Sequence...")
        
        # Step 1: Wait for PX4 to be ready
        rospy.loginfo("⏳ Waiting for PX4 to be ready...")
        while not rospy.is_shutdown():
            if self.current_state.connected and self.current_state.mode:
                break
            rospy.loginfo("Waiting for PX4 connection...")
            time.sleep(1)
            
        # Step 2: Send some setpoints before switching to OFFBOARD
        rospy.loginfo("📡 Sending initial setpoints...")
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = self.target_altitude
        
        # Send setpoints for 2 seconds
        for i in range(100):
            if rospy.is_shutdown():
                return
            self.local_pos_pub.publish(pose)
            self.rate.sleep()
            
        # Step 3: Set OFFBOARD mode
        rospy.loginfo("🎮 Setting OFFBOARD mode...")
        offboard_set = False
        for i in range(10):
            if self.set_mode("OFFBOARD"):
                offboard_set = True
                break
            time.sleep(0.5)
            
        if not offboard_set:
            rospy.logerr("❌ Failed to set OFFBOARD mode")
            return
            
        # Step 4: ARM the vehicle
        rospy.loginfo("🔥 Arming vehicle...")
        armed = False
        for i in range(10):
            if self.current_state.armed:
                armed = True
                rospy.loginfo("✅ Vehicle already armed")
                break
            if self.arm_vehicle():
                armed = True
                break
            time.sleep(0.5)
            
        if not armed:
            rospy.logerr("❌ Failed to arm vehicle")
            return
            
        # Step 5: Continue sending setpoints and monitor altitude
        rospy.loginfo(f"🚀 Taking off to {self.target_altitude}m...")
        
        while not rospy.is_shutdown() and not self.takeoff_complete:
            # Continue sending position setpoints
            self.local_pos_pub.publish(pose)
            
            # Print current altitude every 2 seconds
            current_alt = self.current_pose.pose.position.z
            if int(rospy.get_time()) % 2 == 0:
                rospy.loginfo(f"Current altitude: {current_alt:.2f}m / Target: {self.target_altitude}m")
                
            self.rate.sleep()
            
        # Step 6: Hold position after takeoff
        rospy.loginfo("✅ Takeoff complete! Holding position...")
        while not rospy.is_shutdown():
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        takeoff_node = AutoTakeoff()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Auto takeoff node terminated")
        pass
