#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest, CommandTOLRequest, CommandTOL
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

# ArduPilot specific flight modes
GUIDED_MODE = "GUIDED"
# GUIDED_NOGPS_MODE = "GUIDED_NOGPS"
GUIDED_NOGPS_MODE = "GUIDED"
LAND_MODE = "LAND"

class TakeoffHoverController:
    def __init__(self):
        rospy.init_node('ardupilot_takeoff_hover', anonymous=True)
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
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        # State variables
        self.takeoff_complete = False
        self.guided_mode_active = False
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.extended_state = ExtendedState()
        self.current_teb_status = String()
        # Configuration
        self.takeoff_target_altitude = 1.0
        self.altitude_tolerance = 0.1
        # Control flags
        self.hovering = False
        rospy.loginfo("[ArduPilot Takeoff] Controller initialized")

    def teb_status_cb(self, msg):
        self.current_teb_status = msg
        self.hovering = (msg.data == "HOVERING")
        if not self.hovering:
            rospy.loginfo("[takeoff] Hovering stopped")

    def state_cb(self, msg):
        self.current_state = msg
        self.guided_mode_active = (msg.mode==GUIDED_MODE)

    def pose_cb(self, msg):
        self.current_pose = msg

    def wait_for_mavros_connection(self, timeout=30):
        rospy.loginfo("[ArduPilot Takeoff] Waiting for MAVROS connection...")
        rate = rospy.Rate(20)
        start_time = time.time()
        while not rospy.is_shutdown() and not self.current_state.connected:
            if time.time() - start_time > timeout:
                rospy.logerr("[ArduPilot Takeoff] Timeout waiting for MAVROS connection")
                return False
            rate.sleep()

        rospy.loginfo("[ArduPilot Takeoff] MAVROS connected")
        return True

    def set_guided_mode(self):
        if not self.current_state.connected:
            rospy.logwarn("[ArduPilot Takeoff] Vehicle not connected")
            return False
        target_mode = GUIDED_MODE
        if self.current_state.mode != target_mode:
            try:
                resp = self.set_mode_client(0, target_mode)
                if resp.mode_sent:
                    rospy.loginfo(f"[ArduPilot Takeoff] {target_mode} mode set")
                    return True
                else:
                    rospy.logwarn(f"[ArduPilot Takeoff] Failed to set {target_mode} mode")
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(f"[ArduPilot Takeoff] Service call failed: {e}")
                return False
        return True

    def arm_vehicle(self):
        """Arm the vehicle"""
        if not self.current_state.armed:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True
            try:
                if self.arming_client.call(arm_cmd).success:
                    rospy.loginfo("[ArduPilot Takeoff] Vehicle armed")
                    return True
                else:
                    rospy.logwarn("[ArduPilot Takeoff] Failed to arm vehicle")
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(f"[ArduPilot Takeoff] Arming service failed: {e}")
                return False
        return True

    def takeoff_command(self):
        """Send ArduPilot takeoff command"""
        try:
            takeoff_cmd = CommandTOLRequest()
            takeoff_cmd.min_pitch = 0.0
            takeoff_cmd.yaw = 0.0
            takeoff_cmd.latitude = 0.0  # Use current position
            takeoff_cmd.longitude = 0.0
            takeoff_cmd.altitude = self.takeoff_target_altitude
            resp = self.takeoff_client.call(takeoff_cmd)
            if resp.success:
                rospy.loginfo(f"[ArduPilot Takeoff] Takeoff command sent to {self.takeoff_target_altitude}m")
                return True
            else:
                rospy.logwarn("[ArduPilot Takeoff] Takeoff command failed")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"[ArduPilot Takeoff] Takeoff service failed: {e}")
            return False

    # def send_position_setpoint(self, x=None, y=None, z=None):
    #     """Send position setpoint for hover control"""
    #     pose = PoseStamped()
    #     pose.header.stamp = rospy.Time.now()
    #     pose.header.frame_id = "map"
    #     # Use current position if not specified
    #     if x is None:
    #         x = self.current_pose.pose.position.x
    #     if y is None:
    #         y = self.current_pose.pose.position.y
    #     if z is None:
    #         z = self.takeoff_altitude
    #     pose.pose.position.x = x
    #     pose.pose.position.y = y
    #     pose.pose.position.z = z
    #     # Maintain current orientation
    #     pose.pose.orientation = self.current_pose.pose.orientation
    #     self.local_position_pub.publish(pose)

    def check_altitude_reached(self):
        """Check if target altitude is reached"""
        current_alt = self.current_pose.pose.position.z
        target_reached = abs(current_alt - self.takeoff_target_altitude) < self.altitude_tolerance
        if target_reached:
            rospy.loginfo(f"[ArduPilot Takeoff] Altitude reached: {current_alt:.2f}m (target: {self.takeoff_target_altitude}m)")
        return target_reached

    def wait_for_altitude(self, timeout=60):
        """Wait for drone to reach target altitude"""
        rospy.loginfo(f"[ArduPilot Takeoff] Waiting for altitude {self.takeoff_target_altitude}m...")
        start_time = time.time()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.check_altitude_reached():
                rospy.loginfo("[ArduPilot Takeoff] Target altitude reached!")
                return True
            if time.time() - start_time > timeout:
                rospy.logerr("[ArduPilot Takeoff] Altitude timeout!")
                return False
            # Check if still armed
            if not self.current_state.armed:
                rospy.logerr("[ArduPilot Takeoff] Vehicle disarmed during takeoff!")
                return False
            # Log current altitude every 5 seconds
            if int(time.time() - start_time) % 5 == 0:
                current_alt = self.current_pose.pose.position.z
                rospy.loginfo(f"[ArduPilot Takeoff] Current altitude: {current_alt:.2f}m")
            rate.sleep()
        return False

    def takeoff_sequence(self):
        """Execute complete ArduPilot takeoff sequence"""
        rospy.loginfo("[ArduPilot Takeoff] Starting ArduPilot takeoff sequence...")
        if not self.wait_for_mavros_connection():
            return False
        rospy.loginfo("[ArduPilot Takeoff] Setting GUIDED mode...")
        if not self.set_guided_mode():
            return False
        rate = rospy.Rate(10)
        timeout = time.time() + 10
        target_mode = GUIDED_MODE
        while self.current_state.mode != target_mode and time.time() < timeout:
            rate.sleep()
        if self.current_state.mode != target_mode:
            rospy.logerr(f"[ArduPilot Takeoff] Failed to enter {target_mode} mode")
            return False
        rospy.loginfo("[ArduPilot Takeoff] Arming vehicle...")
        if not self.arm_vehicle():
            return False
        timeout = time.time() + 10
        while not self.current_state.armed and time.time() < timeout:
            rate.sleep()
        if not self.current_state.armed:
            rospy.logerr("[ArduPilot Takeoff] Failed to arm vehicle")
            return False
        rospy.loginfo("[ArduPilot Takeoff] Sending takeoff command...")
        if not self.takeoff_command():
            return False
        if not self.wait_for_altitude():
            return False
        rospy.loginfo("[ArduPilot Takeoff] Stabilizing at target altitude...")
        stabilize_start = time.time()
        while time.time() - stabilize_start < 3.0:  # 3 second stabilization
            self.send_taget_takeoff(self.takeoff_target_altitude)
            rate.sleep()
        self.takeoff_complete = True
        rospy.loginfo("[ArduPilot Takeoff] Takeoff sequence completed successfully!")
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
