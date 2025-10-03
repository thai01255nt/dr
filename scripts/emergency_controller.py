#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String, Bool
import threading
import time

class EmergencyController:
    def __init__(self):
        rospy.init_node('emergency_controller', anonymous=True)
        
        # Publishers
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.emergency_status_pub = rospy.Publisher('/emergency_controller/status', String, queue_size=10)
        self.emergency_active_pub = rospy.Publisher('/emergency_controller/active', Bool, queue_size=10)
        
        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.extended_state_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extended_state_cb)
        
        # Service clients
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        # Services
        self.emergency_land_srv = rospy.Service('/emergency_controller/emergency_land', Trigger, self.emergency_land_callback)
        self.emergency_hover_srv = rospy.Service('/emergency_controller/emergency_hover', Trigger, self.emergency_hover_callback)
        self.return_to_launch_srv = rospy.Service('/emergency_controller/return_to_launch', Trigger, self.return_to_launch_callback)
        self.emergency_stop_srv = rospy.Service('/emergency_controller/emergency_stop', Trigger, self.emergency_stop_callback)
        self.reset_emergency_srv = rospy.Service('/emergency_controller/reset', Trigger, self.reset_emergency_callback)
        
        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.extended_state = ExtendedState()
        self.emergency_active = False
        self.emergency_type = None
        self.launch_position = None
        self.emergency_hover_position = None
        
        # Safety parameters
        self.min_altitude = 0.5  # Minimum safe altitude
        self.max_altitude = 10.0  # Maximum safe altitude
        self.battery_critical_threshold = 20.0  # Critical battery percentage
        self.connection_timeout = 5.0  # Seconds before connection loss emergency
        
        # Monitoring
        self.last_heartbeat = time.time()
        self.monitoring_thread = threading.Thread(target=self.safety_monitoring_loop)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        
        rospy.loginfo("Emergency Controller initialized")

    def state_cb(self, msg):
        self.current_state = msg
        self.last_heartbeat = time.time()
        
        # Store launch position when first armed
        if msg.armed and self.launch_position is None:
            self.launch_position = PoseStamped()
            self.launch_position.header = self.current_pose.header
            self.launch_position.pose = self.current_pose.pose
            rospy.loginfo(f"Launch position recorded: ({self.launch_position.pose.position.x:.2f}, {self.launch_position.pose.position.y:.2f})")

    def pose_cb(self, msg):
        self.current_pose = msg

    def extended_state_cb(self, msg):
        self.extended_state = msg

    def publish_emergency_status(self, status):
        """Publish emergency status"""
        msg = String()
        msg.data = status
        self.emergency_status_pub.publish(msg)
        
        active_msg = Bool()
        active_msg.data = self.emergency_active
        self.emergency_active_pub.publish(active_msg)
        
        rospy.logwarn(f"Emergency Status: {status}")

    def emergency_land_callback(self, req):
        """Emergency landing service"""
        try:
            rospy.logwarn("EMERGENCY LANDING INITIATED!")
            self.emergency_active = True
            self.emergency_type = "LAND"
            
            # Set LAND mode
            land_mode = SetMode()
            land_mode.custom_mode = 'LAND'
            
            if self.set_mode_client.call(land_mode).mode_sent:
                self.publish_emergency_status("EMERGENCY_LANDING")
                return TriggerResponse(success=True, message="Emergency landing initiated")
            else:
                return TriggerResponse(success=False, message="Failed to set LAND mode")
                
        except Exception as e:
            rospy.logerr(f"Emergency land failed: {e}")
            return TriggerResponse(success=False, message=str(e))

    def emergency_hover_callback(self, req):
        """Emergency hover service"""
        try:
            rospy.logwarn("EMERGENCY HOVER INITIATED!")
            self.emergency_active = True
            self.emergency_type = "HOVER"
            
            # Record current position for hover
            self.emergency_hover_position = PoseStamped()
            self.emergency_hover_position.header = self.current_pose.header
            self.emergency_hover_position.pose = self.current_pose.pose
            
            # Ensure OFFBOARD mode
            if self.current_state.mode != "OFFBOARD":
                offboard_mode = SetMode()
                offboard_mode.custom_mode = 'OFFBOARD'
                self.set_mode_client.call(offboard_mode)
            
            self.publish_emergency_status("EMERGENCY_HOVER")
            return TriggerResponse(success=True, message="Emergency hover initiated")
            
        except Exception as e:
            rospy.logerr(f"Emergency hover failed: {e}")
            return TriggerResponse(success=False, message=str(e))

    def return_to_launch_callback(self, req):
        """Return to launch service"""
        try:
            if self.launch_position is None:
                return TriggerResponse(success=False, message="Launch position not recorded")
            
            rospy.logwarn("RETURN TO LAUNCH INITIATED!")
            self.emergency_active = True
            self.emergency_type = "RTL"
            
            # Set RTL mode or use OFFBOARD to go to launch position
            rtl_mode = SetMode()
            rtl_mode.custom_mode = 'RTL'
            
            if self.set_mode_client.call(rtl_mode).mode_sent:
                self.publish_emergency_status("RETURN_TO_LAUNCH")
                return TriggerResponse(success=True, message="Return to launch initiated")
            else:
                # Fallback: use OFFBOARD mode to go to launch position
                offboard_mode = SetMode()
                offboard_mode.custom_mode = 'OFFBOARD'
                if self.set_mode_client.call(offboard_mode).mode_sent:
                    self.emergency_type = "RTL_OFFBOARD"
                    self.publish_emergency_status("RETURN_TO_LAUNCH_OFFBOARD")
                    return TriggerResponse(success=True, message="Return to launch initiated (OFFBOARD)")
                else:
                    return TriggerResponse(success=False, message="Failed to set RTL or OFFBOARD mode")
                
        except Exception as e:
            rospy.logerr(f"Return to launch failed: {e}")
            return TriggerResponse(success=False, message=str(e))

    def emergency_stop_callback(self, req):
        """Emergency stop service - immediate hover"""
        try:
            rospy.logwarn("EMERGENCY STOP INITIATED!")
            self.emergency_active = True
            self.emergency_type = "STOP"
            
            # Send zero velocity
            zero_vel = TwistStamped()
            zero_vel.header.stamp = rospy.Time.now()
            self.velocity_pub.publish(zero_vel)
            
            # Record position for hover
            self.emergency_hover_position = PoseStamped()
            self.emergency_hover_position.header = self.current_pose.header
            self.emergency_hover_position.pose = self.current_pose.pose
            
            self.publish_emergency_status("EMERGENCY_STOP")
            return TriggerResponse(success=True, message="Emergency stop activated")
            
        except Exception as e:
            rospy.logerr(f"Emergency stop failed: {e}")
            return TriggerResponse(success=False, message=str(e))

    def reset_emergency_callback(self, req):
        """Reset emergency state"""
        try:
            rospy.loginfo("Resetting emergency state...")
            self.emergency_active = False
            self.emergency_type = None
            self.emergency_hover_position = None
            
            self.publish_emergency_status("RESET")
            return TriggerResponse(success=True, message="Emergency state reset")
            
        except Exception as e:
            rospy.logerr(f"Reset emergency failed: {e}")
            return TriggerResponse(success=False, message=str(e))

    def send_emergency_hover_setpoint(self):
        """Send hover setpoint during emergency"""
        if self.emergency_hover_position is None:
            return
            
        hover_cmd = PoseStamped()
        hover_cmd.header.stamp = rospy.Time.now()
        hover_cmd.header.frame_id = "map"
        hover_cmd.pose = self.emergency_hover_position.pose
        
        self.position_pub.publish(hover_cmd)

    def send_rtl_setpoint(self):
        """Send RTL setpoint in OFFBOARD mode"""
        if self.launch_position is None:
            return
            
        rtl_cmd = PoseStamped()
        rtl_cmd.header.stamp = rospy.Time.now()
        rtl_cmd.header.frame_id = "map"
        rtl_cmd.pose = self.launch_position.pose
        rtl_cmd.pose.position.z = max(rtl_cmd.pose.position.z, 2.0)  # Ensure safe altitude
        
        self.position_pub.publish(rtl_cmd)

    def check_altitude_safety(self):
        """Check if altitude is within safe bounds"""
        if hasattr(self.current_pose.pose.position, 'z'):
            altitude = self.current_pose.pose.position.z
            
            if altitude < self.min_altitude:
                rospy.logwarn(f"Altitude too low: {altitude:.2f}m")
                return False
            elif altitude > self.max_altitude:
                rospy.logwarn(f"Altitude too high: {altitude:.2f}m")
                return False
        return True

    def check_connection_health(self):
        """Check MAVROS connection health"""
        if time.time() - self.last_heartbeat > self.connection_timeout:
            rospy.logerr("MAVROS connection lost!")
            return False
        return True

    def auto_emergency_triggers(self):
        """Check for automatic emergency triggers"""
        # Connection loss
        if not self.check_connection_health():
            if not self.emergency_active:
                rospy.logerr("Auto-triggering emergency due to connection loss")
                self.emergency_hover_callback(None)
        
        # Altitude bounds
        if not self.check_altitude_safety():
            if not self.emergency_active and self.current_state.armed:
                rospy.logerr("Auto-triggering emergency due to unsafe altitude")
                self.emergency_hover_callback(None)

    def safety_monitoring_loop(self):
        """Main safety monitoring loop"""
        rate = rospy.Rate(10)  # 10Hz monitoring
        
        while not rospy.is_shutdown():
            try:
                # Check for automatic emergency triggers
                if not self.emergency_active:
                    self.auto_emergency_triggers()
                
                # Handle active emergency states
                if self.emergency_active:
                    if self.emergency_type == "HOVER" or self.emergency_type == "STOP":
                        # Ensure OFFBOARD mode and send hover commands
                        if self.current_state.mode != "OFFBOARD":
                            offboard_mode = SetMode()
                            offboard_mode.custom_mode = 'OFFBOARD'
                            self.set_mode_client.call(offboard_mode)
                        
                        self.send_emergency_hover_setpoint()
                        
                    elif self.emergency_type == "RTL_OFFBOARD":
                        # Manual RTL using OFFBOARD mode
                        self.send_rtl_setpoint()
                        
                        # Check if we're close to launch position
                        if self.launch_position is not None:
                            dx = self.current_pose.pose.position.x - self.launch_position.pose.position.x
                            dy = self.current_pose.pose.position.y - self.launch_position.pose.position.y
                            distance = (dx**2 + dy**2)**0.5
                            
                            if distance < 1.0:  # Within 1m of launch position
                                rospy.loginfo("Reached launch position, switching to hover")
                                self.emergency_type = "HOVER"
                                self.emergency_hover_position = self.launch_position
                
            except Exception as e:
                rospy.logerr(f"Safety monitoring error: {e}")
                
            rate.sleep()

def main():
    try:
        controller = EmergencyController()
        
        rospy.loginfo("Emergency Controller running...")
        rospy.loginfo("Available services:")
        rospy.loginfo("  /emergency_controller/emergency_land")
        rospy.loginfo("  /emergency_controller/emergency_hover") 
        rospy.loginfo("  /emergency_controller/return_to_launch")
        rospy.loginfo("  /emergency_controller/emergency_stop")
        rospy.loginfo("  /emergency_controller/reset")
        
        # Keep the node alive
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Emergency Controller interrupted")
    except Exception as e:
        rospy.logerr(f"Emergency Controller error: {e}")

if __name__ == '__main__':
    main()