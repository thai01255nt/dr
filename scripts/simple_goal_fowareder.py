#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class SimpleGoalForwarder:
    def __init__(self):
        rospy.init_node("simple_goal_forwarder")

        # Tạo action client tới move_base
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        # Sub vào goal từ RViz
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

    def goal_callback(self, msg):
        rospy.loginfo("Received goal from RViz -> forwarding to /move_base/goal")

        goal = MoveBaseGoal()
        goal.target_pose.header = msg.header
        goal.target_pose.pose = msg.pose

        # Gửi goal qua actionlib
        self.client.send_goal(goal,
                              done_cb=self.done_cb,
                              active_cb=self.active_cb,
                              feedback_cb=self.feedback_cb)

    def active_cb(self):
        rospy.loginfo("Goal is now active")

    def feedback_cb(self, feedback):
        # Feedback chứa vị trí hiện tại mà planner đang hướng tới
        rospy.loginfo_throttle(5, "Feedback received...")

    def done_cb(self, status, result):
        if status == 3:
            rospy.loginfo("Goal reached successfully ✅")
        elif status == 4:
            rospy.logwarn("Goal was aborted ❌")
        elif status == 2:
            rospy.logwarn("Goal was preempted (cancelled)")
        else:
            rospy.logwarn("Goal ended with status: %d" % status)

if __name__ == "__main__":
    try:
        node = SimpleGoalForwarder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

