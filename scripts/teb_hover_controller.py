#!/usr/bin/env python3

import rospy
import threading
import time
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, ExtendedState
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, SetModeRequest, CommandBool, CommandTOL, CommandBoolRequest
from std_msgs.msg import String, Bool
import math

class TEBHoverController:
    def __init__(self):
        rospy.init_node('teb_hover_controller', anonymous=True)
        # Publishers
        self.local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.status_pub = rospy.Publisher('/teb_controller/status', String, queue_size=10)
        self.goal_reached_pub = rospy.Publisher('/teb_controller/goal_reached', Bool, queue_size=10)
        self.extended_state_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extended_state_cb)
        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.cmd_vel_sub = rospy.Subscriber('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, self.cmd_vel_cb)
        self.goal_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_cb)
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_cb)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_cb)
        self.global_plan_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_plan_cb)
        # Service clients
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
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
        # takeoff
        self.takeoff_complete = False
        self.offboard_enabled = False

        self.takeoff_target_altitude = 1.0
        self.takeoff_target_tolerance = 0.1
        # hover
        self.hovering = False

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
        self.local_position_pub.publish(hover_cmd)

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

    def wait_for_mavros_connection(self, timeout=30):
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
    def send_taget_takeoff(self, target_altitude):
        """Send position setpoint"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = target_altitude
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        self.local_position_pub.publish(pose)

    def check_altitude_reached(self):
        """Check if target altitude is reached"""
        current_alt = self.current_pose.pose.position.z
        return abs(current_alt - self.takeoff_target_altitude) < self.takeoff_target_tolerance

    def takeoff_sequence(self):
        rospy.loginfo("[Takeoff Controller] Starting takeoff sequence...")
        if not self.wait_for_mavros_connection():
            return False
        rospy.loginfo("[Takeoff Controller] Sending initial setpoints...")
        rate = rospy.Rate(20)
        for i in range(100):
            if rospy.is_shutdown():
                return False
            self.send_taget_takeoff(self.takeoff_target_altitude)
            rate.sleep()
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
        rospy.loginfo(f"[Takeoff Controller] Taking off to {self.takeoff_target_altitude}m altitude...")
        start_time = time.time()
        takeoff_timeout = 30  # 30 seconds timeout
        while not rospy.is_shutdown():
            self.send_taget_takeoff(self.takeoff_target_altitude)
            if self.check_altitude_reached():
                rospy.loginfo(f"[Takeoff Controller] Target altitude {self.takeoff_target_altitude}m reached!")
                self.takeoff_complete = True
                self.offboard_enabled = True
                rospy.loginfo("[Takeoff Controller] Stabilizing at target altitude...")
                stabilize_start = time.time()
                while time.time() - stabilize_start < 2.0:
                    self.send_taget_takeoff(self.takeoff_target_altitude)
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

    def emergency_stop(self):
        """Emergency stop - hover at current position"""
        rospy.logwarn("[TEB Controller] Emergency stop activated!")
        self.is_goal_active = False
        self.teb_active = False
        self.start_hovering()
        self.publish_status("EMERGENCY_STOP")

    def hover(self):
        if not self.takeoff_complete:
            rospy.logwarn("[Hover] Takeoff not complete")
            return
        rate = rospy.Rate(20)
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        current_z = self.takeoff_target_altitude
        current_orientation_x = self.current_pose.pose.orientation.x
        current_orientation_y = self.current_pose.pose.orientation.y
        current_orientation_z = self.current_pose.pose.orientation.z
        while not rospy.is_shutdown() and self.hovering:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = current_x
            pose.pose.position.y = current_y
            pose.pose.position.z = current_z
            pose.pose.orientation.x = current_orientation_x
            pose.pose.orientation.y = current_orientation_y
            pose.pose.orientation.z = current_orientation_z
            self.local_position_pub.publish(pose)
            rate.sleep()

def main():
    try:
        controller = TEBHoverController()
        if controller.takeoff_sequence():
            rospy.loginfo("[takeoff] Takeoff completed successfully!")

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
