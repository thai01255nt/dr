#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest
from std_msgs.msg import String

class TakeoffHoverController:
    def __init__(self):
        rospy.init_node('takeoff_hover_controller', anonymous=True)
        # Publishers
        self.local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.teb_status_sub = rospy.Subscriber('/teb_controller/status', String, self.teb_status_cb)
        # Service clients
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        # takeoff
        self.takeoff_complete = False
        self.offboard_enabled = False
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_teb_status = String()
        self.takeoff_target_altitude = 1.0
        self.takeoff_target_tolerance = 0.1
        # hover
        self.hovering = False

    def teb_status_cb(self, msg):
        self.current_teb_status = msg
        self.hovering = (msg.data == "HOVERING")
        if not self.hovering:
            rospy.loginfo("[takeoff] Hovering stopped")

    def state_cb(self, msg):
        self.current_state = msg
        self.offboard_mode_active = (msg.mode == "OFFBOARD")

    def pose_cb(self, msg):
        self.current_pose = msg

    def wait_for_mavros_connection(self, timeout=30):
        rospy.loginfo("[takeoff] Waiting for MAVROS connection...")
        rate = rospy.Rate(20)
        start_time = time.time()
        while not rospy.is_shutdown() and not self.current_state.connected:
            if time.time() - start_time > timeout:
                rospy.logerr("[takeoff] Timeout waiting for MAVROS connection")
                return False
            rate.sleep()
        rospy.loginfo("[takeoff] MAVROS connected")
        return True

    def set_offboard_mode(self):
        """Set vehicle to OFFBOARD mode"""
        if not self.current_state.connected:
            rospy.logwarn("[takeoff] Vehicle not connected")
            return False
        if self.current_state.mode != "OFFBOARD":
            try:
                resp = self.set_mode_client(0, "OFFBOARD")
                if resp.mode_sent:
                    rospy.loginfo("[takeoff] OFFBOARD mode set")
                    return True
                else:
                    rospy.logwarn("[takeoff] Failed to set OFFBOARD mode")
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(f"[takeoff] Service call failed: {e}")
                return False
        return True

    def arm_vehicle(self):
        if not self.current_state.armed:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True
            if self.arming_client.call(arm_cmd).success:
                rospy.loginfo("[takeoff] Vehicle armed")
                return True
            else:
                rospy.logwarn("[takeoff] Failed to arm vehicle")
                return False
        return True

    def send_taget_takeoff(self, target_altitude):
        """Send position setpoint"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
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
        rospy.loginfo("[takeoff] Starting takeoff sequence...")
        if not self.wait_for_mavros_connection():
            return False
        rospy.loginfo("[takeoff] Sending initial setpoints...")
        rate = rospy.Rate(20)
        for i in range(100):
            if rospy.is_shutdown():
                return False
            self.send_taget_takeoff(self.takeoff_target_altitude)
            rate.sleep()
        rospy.loginfo("[takeoff] Setting OFFBOARD mode...")
        if not self.set_offboard_mode():
            return False
        # Wait for mode to be set
        timeout = time.time() + 5
        while self.current_state.mode != "OFFBOARD" and time.time() < timeout:
            rate.sleep()
        if self.current_state.mode != "OFFBOARD":
            rospy.logerr("[takeoff] Failed to enter OFFBOARD mode")
            return False
        # Arm vehicle
        rospy.loginfo("[takeoff] Arming vehicle...")
        if not self.arm_vehicle():
            return False
        # Wait for arming
        timeout = time.time() + 5
        while not self.current_state.armed and time.time() < timeout:
            rate.sleep()
        if not self.current_state.armed:
            rospy.logerr("[takeoff] Failed to arm vehicle")
            return False
        # Takeoff to target altitude
        rospy.loginfo(f"[takeoff] Taking off to {self.takeoff_target_altitude}m altitude...")
        start_time = time.time()
        takeoff_timeout = 30  # 30 seconds timeout
        while not rospy.is_shutdown():
            self.send_taget_takeoff(self.takeoff_target_altitude)
            if self.check_altitude_reached():
                rospy.loginfo(f"[takeoff] Target altitude {self.takeoff_target_altitude}m reached!")
                self.takeoff_complete = True
                self.offboard_enabled = True
                rospy.loginfo("[takeoff] Stabilizing at target altitude...")
                stabilize_start = time.time()
                while time.time() - stabilize_start < 2.0:
                    self.send_taget_takeoff(self.takeoff_target_altitude)
                    rate.sleep()
                return True
            # Check timeout
            if time.time() - start_time > takeoff_timeout:
                rospy.logerr("[takeoff] Takeoff timeout!")
                return False
            # Check if still armed and in offboard
            if not self.current_state.armed:
                rospy.logerr("[takeoff] Vehicle disarmed during takeoff!")
                return False
            if self.current_state.mode != "OFFBOARD":
                rospy.logerr("[takeoff] Lost OFFBOARD mode during takeoff!")
                return False
            rate.sleep()
        return False

    def start_hover(self):
        if not self.takeoff_complete:
            rospy.logwarn("[Hover] Takeoff not complete")
            return
        rospy.loginfo("[Hover] Hovering...")
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
        controller = TakeoffHoverController()
        if controller.takeoff_sequence():
            rospy.loginfo("[takeoff] Takeoff completed successfully!")
            controller.hovering = True
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                if controller.hovering:
                    controller.start_hover()
                rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.loginfo(f"[takeoff] interrupted with error: {e}")
    except Exception as e:
        rospy.logerr(f"[takeoff] error: {e}")

if __name__ == '__main__':
    main()
