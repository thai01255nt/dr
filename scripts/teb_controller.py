#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseStamped, TwistStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from mavros_msgs.msg import State
from std_msgs.msg import String, Bool

class TEBController:
    def __init__(self):
        rospy.init_node('teb_controller', anonymous=True)
        # Publishers
        self.local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.status_pub = rospy.Publisher('/teb_controller/status', String, queue_size=10)
        self.goal_reached_pub = rospy.Publisher('/teb_controller/goal_reached', Bool, queue_size=10)
        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.goal_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_cb)
        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.goal_pose = None
        self.last_cmd_vel_time = 0
        self.cmd_vel_timeout = 0.5
        self.is_goal_active = False
        self.goal_reached = False
        # Control flags
        self.teb_active = False
        self.offboard_mode_active = False
        # Velocity monitoring
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
        # self.publish_status("TEB_CONTROLLING")

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
                self.publish_status("GOAL_ACTIVE")
                rospy.loginfo("[TEB Controller] New goal activated")
        elif latest_status.status == GoalStatus.SUCCEEDED:
            if self.is_goal_active:
                self.goal_reached = True
                self.is_goal_active = False
                self.teb_active = False
                self.publish_status("HOVERING")
                rospy.loginfo("[TEB Controller] Goal reached successfully!")
        # elif latest_status.status in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED]:
        #     if self.is_goal_active:
        #         self.is_goal_active = False
        #         self.teb_active = False
        #         self.start_hovering()
        #         self.publish_status(f"HOVERING")
        #         rospy.logwarn(f"[TEB Controller] Goal ended with status: {latest_status.status}")
        #
    # def goal_cb(self, msg):
    #     """Handle new goal"""
    #     self.goal_pose = msg.goal.target_pose
    #     self.is_goal_active = True
    #     self.goal_reached = False
    #     self.teb_active = True
    #     self.hovering = False
    #     rospy.loginfo(f"[TEB Controller] New goal received: ({self.goal_pose.pose.position.x:.2f}, {self.goal_pose.pose.position.y:.2f})")

    # def result_cb(self, msg):
    #     """Handle move_base result"""
    #     if msg.status.status == GoalStatus.SUCCEEDED:
    #         self.goal_reached = True
    #         self.start_hovering()

    # def global_plan_cb(self, msg):
    #     """Monitor global plan"""
    #     if len(msg.poses) > 0:
    #         # Plan exists, TEB should be active
    #         pass

    def publish_status(self, status):
        """Publish controller status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main():
    try:
        TEBController()
        rospy.loginfo("[TEB Controller] TEB Controller running...")
        # Keep the node alive
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[TEB Controller] TEB Hover Controller interrupted")
    except Exception as e:
        rospy.logerr(f"[TEB Controller] TEB Hover Controller error: {e}")

if __name__ == '__main__':
    main()
