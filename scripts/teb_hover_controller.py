#!/usr/bin/env python3

import rospy
import threading
import time
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, SetModeRequest
from std_msgs.msg import String, Bool
import math

class TEBHoverController:
    def __init__(self):
        rospy.init_node('teb_hover_controller', anonymous=True)
        
        # Publishers
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.status_pub = rospy.Publisher('/teb_controller/status', String, queue_size=10)
        self.goal_reached_pub = rospy.Publisher('/teb_controller/goal_reached', Bool, queue_size=10)
        
        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.cmd_vel_sub = rospy.Subscriber('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, self.cmd_vel_cb)
        self.goal_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_cb)
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_cb)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_cb)
        self.global_plan_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_plan_cb)
        
        # Service clients
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.goal_pose = None
        self.last_cmd_vel_time = 0
        self.cmd_vel_timeout = 2.0  # 2 seconds without cmd_vel = goal reached
        self.is_goal_active = False
        self.goal_reached = False
        self.hover_position = None
        
        # Control flags
        self.teb_active = False
        self.hovering = False
        self.offboard_mode_active = False
        
        # Velocity monitoring
        self.last_velocity = Twist()
        self.velocity_threshold = 0.05  # m/s - consider stopped if below this
        self.zero_velocity_count = 0
        self.zero_velocity_threshold = 20  # 20 consecutive zero velocity readings
        
        # Threading
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        rospy.loginfo("[TEB Controller] TEB Controller initialized")

    def state_cb(self, msg):
        self.current_state = msg
        self.offboard_mode_active = (msg.mode == "OFFBOARD")

    def pose_cb(self, msg):
        self.current_pose = msg

    def cmd_vel_cb(self, msg):
        """Monitor cmd_vel from TEB planner"""
        self.last_cmd_vel_time = time.time()
        self.last_velocity = msg
        
        # Check if velocity is near zero
        vel_magnitude = math.sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.angular.z**2)
        
        if vel_magnitude < self.velocity_threshold:
            self.zero_velocity_count += 1
        else:
            self.zero_velocity_count = 0
            
        # If TEB is sending commands, it's active
        if vel_magnitude > self.velocity_threshold:
            self.teb_active = True
            if self.hovering:
                self.hovering = False
                self.publish_status("TEB_CONTROLLING")

    def goal_status_cb(self, msg):
        """Monitor move_base goal status"""
        if not msg.status_list:
            return
            
        latest_status = msg.status_list[-1]
        
        if latest_status.status == GoalStatus.ACTIVE:
            if not self.is_goal_active:
                self.is_goal_active = True
                self.goal_reached = False
                self.teb_active = True
                self.hovering = False
                self.publish_status("GOAL_ACTIVE")
                rospy.loginfo("[TEB Controller] New goal activated")
                
        elif latest_status.status == GoalStatus.SUCCEEDED:
            if self.is_goal_active:
                self.goal_reached = True
                self.is_goal_active = False
                self.teb_active = False
                self.start_hovering()
                self.publish_status("GOAL_REACHED")
                rospy.loginfo("[TEB Controller] Goal reached successfully!")
                
        elif latest_status.status in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED]:
            if self.is_goal_active:
                self.is_goal_active = False
                self.teb_active = False
                self.start_hovering()
                self.publish_status(f"GOAL_{latest_status.status}")
                rospy.logwarn(f"[TEB Controller] Goal ended with status: {latest_status.status}")

    def goal_cb(self, msg):
        """Handle new goal"""
        self.goal_pose = msg.goal.target_pose
        self.is_goal_active = True
        self.goal_reached = False
        self.teb_active = True
        self.hovering = False
        rospy.loginfo(f"[TEB Controller] New goal received: ({self.goal_pose.pose.position.x:.2f}, {self.goal_pose.pose.position.y:.2f})")

    def result_cb(self, msg):
        """Handle move_base result"""
        if msg.status.status == GoalStatus.SUCCEEDED:
            self.goal_reached = True
            self.start_hovering()

    def global_plan_cb(self, msg):
        """Monitor global plan"""
        if len(msg.poses) > 0:
            # Plan exists, TEB should be active
            pass

    def start_hovering(self):
        """Start hovering at current position"""
        if not self.hovering:
            self.hover_position = PoseStamped()
            self.hover_position.header = self.current_pose.header
            self.hover_position.pose = self.current_pose.pose
            self.hovering = True
            self.teb_active = False
            
            # Publish goal reached
            goal_reached_msg = Bool()
            goal_reached_msg.data = True
            self.goal_reached_pub.publish(goal_reached_msg)
            
            self.publish_status("HOVERING")
            rospy.loginfo(f"[TEB Controller] Started hovering at position: ({self.hover_position.pose.position.x:.2f}, {self.hover_position.pose.position.y:.2f}, {self.hover_position.pose.position.z:.2f})")

    def send_hover_setpoint(self):
        """Send position setpoint to maintain hover"""
        if self.hover_position is None:
            return
            
        hover_cmd = PoseStamped()
        hover_cmd.header.stamp = rospy.Time.now()
        hover_cmd.header.frame_id = "map"
        hover_cmd.pose = self.hover_position.pose
        
        self.position_pub.publish(hover_cmd)

    def send_zero_velocity(self):
        """Send zero velocity command"""
        zero_vel = TwistStamped()
        zero_vel.header.stamp = rospy.Time.now()
        zero_vel.header.frame_id = "base_link"
        # All velocities are zero by default
        self.velocity_pub.publish(zero_vel)

    def ensure_offboard_mode(self):
        """Ensure drone stays in OFFBOARD mode"""
        if not self.offboard_mode_active:
            try:
                mode_cmd = SetModeRequest()
                mode_cmd.custom_mode = 'OFFBOARD'
                
                if self.set_mode_client.call(mode_cmd).mode_sent:
                    rospy.loginfo("[TEB Controller] Switched back to OFFBOARD mode")
                else:
                    rospy.logwarn("[TEB Controller] Failed to switch to OFFBOARD mode")
            except Exception as e:
                rospy.logerr(f"[TEB Controller] Error setting OFFBOARD mode: {e}")

    def publish_status(self, status):
        """Publish controller status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def check_teb_timeout(self):
        """Check if TEB has stopped sending commands"""
        if self.teb_active and time.time() - self.last_cmd_vel_time > self.cmd_vel_timeout:
            # TEB hasn't sent commands for a while, assume goal reached
            if self.is_goal_active:
                rospy.loginfo("[TEB Controller] TEB timeout detected - assuming goal reached")
                self.goal_reached = True
                self.start_hovering()
            return True
        return False

    def check_velocity_timeout(self):
        """Check if velocity has been zero for too long"""
        if (self.teb_active and 
            self.zero_velocity_count >= self.zero_velocity_threshold and
            self.is_goal_active):
            rospy.loginfo("[TEB Controller] Zero velocity detected - assuming goal reached")
            self.goal_reached = True
            self.start_hovering()
            return True
        return False

    def control_loop(self):
        """Main control loop"""
        rate = rospy.Rate(20)  # 20Hz
        
        while not rospy.is_shutdown():
            try:
                # Ensure OFFBOARD mode
                self.ensure_offboard_mode()
                
                # Check for timeouts
                self.check_teb_timeout()
                self.check_velocity_timeout()
                
                # Control logic
                if self.hovering:
                    # Send hover setpoint
                    self.send_hover_setpoint()
                elif not self.teb_active and not self.is_goal_active:
                    # No active goal, maintain current position
                    if self.hover_position is None:
                        self.start_hovering()
                    else:
                        self.send_hover_setpoint()
                elif self.teb_active:
                    # TEB is controlling, just ensure we stay in OFFBOARD
                    # TEB commands are automatically forwarded via move_base remapping
                    pass
                else:
                    # Fallback: send zero velocity
                    self.send_zero_velocity()
                    
            except Exception as e:
                rospy.logerr(f"[TEB Controller] Control loop error: {e}")
                
            rate.sleep()

    def emergency_stop(self):
        """Emergency stop - hover at current position"""
        rospy.logwarn("[TEB Controller] Emergency stop activated!")
        self.is_goal_active = False
        self.teb_active = False
        self.start_hovering()
        self.publish_status("EMERGENCY_STOP")

def main():
    try:
        controller = TEBHoverController()
        
        rospy.loginfo("[TEB Controller] TEB Hover Controller running...")
        rospy.loginfo("[TEB Controller] Monitoring TEB planner and managing hover behavior")
        
        # Keep the node alive
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("[TEB Controller] TEB Hover Controller interrupted")
    except Exception as e:
        rospy.logerr(f"[TEB Controller] TEB Hover Controller error: {e}")

if __name__ == '__main__':
    main()
