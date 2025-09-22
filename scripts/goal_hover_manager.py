#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

class GoalHoverManager:
    def __init__(self):
        rospy.init_node('goal_hover_manager')
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', 
                                          Twist, queue_size=1)
        
        # Subscribers  
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.result_sub = rospy.Subscriber('/move_base/result', 
                                          MoveBaseActionResult, self.result_callback)
        self.status_sub = rospy.Subscriber('/move_base/status',
                                          GoalStatusArray, self.status_callback)
        
        # Service
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # State variables
        self.current_state = State()
        self.goal_reached = False
        self.keep_hovering = False
        self.goal_active = False
        self.last_goal_id = ""
        
        # Timer để gửi cmd_vel=0 liên tục
        self.hover_timer = rospy.Timer(rospy.Duration(0.1), self.hover_callback)  # 10Hz
        
        rospy.loginfo("🚁 Goal Hover Manager started")
        rospy.loginfo("   • Will maintain OFFBOARD mode after goal reached")
        rospy.loginfo("   • Will pause hover when new goal is set")
        rospy.loginfo("   • Ready to manage goal hovering!")
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def status_callback(self, msg):
        """Monitor move_base status để detect goal mới"""
        if msg.status_list:
            latest_status = msg.status_list[-1]
            current_status = latest_status.status
            current_goal_id = latest_status.goal_id.id
            
            # Detect new goal (different goal ID and ACTIVE)
            if current_status == GoalStatus.ACTIVE and current_goal_id != self.last_goal_id:
                if self.keep_hovering:  # Was hovering, now new goal
                    rospy.loginfo(f"🎯 New goal detected! Pausing hover mode (Goal ID: {current_goal_id[:8]}...)")
                    self.keep_hovering = False
                    self.goal_active = True
                    self.last_goal_id = current_goal_id
                    
            # Goal finished (success, aborted, preempted)
            elif current_status in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED]:
                if self.goal_active:
                    self.goal_active = False
        
    def result_callback(self, msg):
        """Callback khi move_base hoàn thành goal"""
        if msg.status.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ Goal reached! Starting hover mode (OFFBOARD + vel=0)")
            self.goal_reached = True
            self.keep_hovering = True
            self.goal_active = False
            self.ensure_offboard_mode()
            
        elif msg.status.status in [GoalStatus.ABORTED, GoalStatus.PREEMPTED]:
            status_text = "ABORTED" if msg.status.status == GoalStatus.ABORTED else "PREEMPTED"
            rospy.logwarn(f"❌ Goal {status_text}! Stopping hover mode")
            self.keep_hovering = False
            self.goal_reached = False
            self.goal_active = False
                
    def ensure_offboard_mode(self):
        """Đảm bảo drone ở OFFBOARD mode"""
        if self.current_state.mode != "OFFBOARD":
            try:
                rospy.loginfo("🔄 Setting OFFBOARD mode for hovering...")
                mode_resp = self.set_mode_client(0, "OFFBOARD")
                if mode_resp.mode_sent:
                    rospy.loginfo("✅ OFFBOARD mode activated")
                else:
                    rospy.logwarn("⚠️  Failed to set OFFBOARD mode")
            except rospy.ServiceException as e:
                rospy.logerr(f"Set mode service call failed: {e}")
                
    def hover_callback(self, event):
        """Gửi cmd_vel=0 để hover khi cần"""
        if self.keep_hovering:
            # Kiểm tra và duy trì OFFBOARD mode
            if self.current_state.mode != "OFFBOARD":
                rospy.logwarn_throttle(5.0, "⚠️  Lost OFFBOARD mode during hover, attempting to restore...")
                self.ensure_offboard_mode()
                
            # Gửi cmd_vel = 0 để hover
            stop_cmd = Twist()
            # Explicitly set all values to 0
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0  
            stop_cmd.linear.z = 0.0
            stop_cmd.angular.x = 0.0
            stop_cmd.angular.y = 0.0
            stop_cmd.angular.z = 0.0
            
            self.cmd_vel_pub.publish(stop_cmd)
            
            # Log hover status every 5 seconds
            if rospy.get_time() % 5 < 0.1:
                rospy.logdebug(f"🚁 Hovering: Mode={self.current_state.mode}, "
                             f"Armed={self.current_state.armed}")

if __name__ == '__main__':
    try:
        manager = GoalHoverManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🚁 Goal Hover Manager shutting down")

