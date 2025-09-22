#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

class OffboardKeeper:
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
        
        # Timer để gửi cmd_vel=0 liên tục
        self.hover_timer = rospy.Timer(rospy.Duration(0.05), self.hover_callback)  # 10Hz
        
        rospy.loginfo("Goal Hover Manager started - will maintain OFFBOARD mode after goal reached")
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def status_callback(self, msg):
        """Monitor move_base status để detect goal mới"""
        if msg.status_list:
            current_status = msg.status_list[-1].status  # Latest status
            
            if current_status == GoalStatus.ACTIVE:
                if self.keep_hovering:  # Goal mới được set
                    rospy.loginfo("New goal detected! Pausing hover mode")
                    self.keep_hovering = False
                    self.goal_active = True
            elif current_status in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED]:
                self.goal_active = False
        
    def result_callback(self, msg):
        """Callback khi move_base hoàn thành"""
        if msg.status.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached! Starting hover mode (vel=0, OFFBOARD maintained)")
            self.goal_reached = True
            self.keep_hovering = True
            self.ensure_offboard_mode()
            
        elif msg.status.status == GoalStatus.ACTIVE:
            # Goal mới được gửi, tạm dừng hover để move_base điều khiển
            if self.keep_hovering:
                rospy.loginfo("New goal active, pausing hover mode")
                self.keep_hovering = False
                
    def ensure_offboard_mode(self):
        """Đảm bảo drone ở OFFBOARD mode"""
        if self.current_state.mode != "OFFBOARD":
            try:
                rospy.loginfo("Setting OFFBOARD mode...")
                mode_resp = self.set_mode_client(0, "OFFBOARD")
                if mode_resp.mode_sent:
                    rospy.loginfo("OFFBOARD mode activated")
                else:
                    rospy.logwarn("Failed to set OFFBOARD mode")
            except rospy.ServiceException as e:
                rospy.logerr(f"Set mode service call failed: {e}")
                
    def hover_callback(self, event):
        """Gửi cmd_vel=0 để hover khi cần"""
        if self.keep_hovering:
            # Kiểm tra và duy trì OFFBOARD mode
            if self.current_state.mode != "OFFBOARD":
                rospy.logwarn_throttle(2.0, "Lost OFFBOARD mode, attempting to restore...")
                self.ensure_offboard_mode()
                
            # Gửi cmd_vel = 0 để hover
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0  
            stop_cmd.linear.z = 0.0
            stop_cmd.angular.x = 0.0
            stop_cmd.angular.y = 0.0
            stop_cmd.angular.z = 0.0
            
            self.cmd_vel_pub.publish(stop_cmd)

if __name__ == '__main__':
    try:
        keeper = OffboardKeeper()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("OFFBOARD Keeper shutting down")

