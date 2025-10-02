#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest, CommandTOL
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

# ArduPilot specific flight modes
GUIDED_MODE = "GUIDED"
GUIDED_NOGPS_MODE = "GUIDED_NOGPS"
LAND_MODE = "LAND"

class TakeoffHoverController:
    def __init__(self):
        rospy.init_node('ardupilot_takeoff_hover', anonymous=True)
        # Publishers
        self.local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.status_pub = rospy.Publisher('/ardupilot_takeoff/status', String, queue_size=10)
        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.extended_state_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extended_state_cb)
        self.global_pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_pos_cb)
        self.teb_status_sub = rospy.Subscriber('/teb_controller/status', String, self.teb_status_cb)
        # Service clients
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.current_teb_status = String()
        # Configuration
        self.takeoff_altitude = 1.0
        self.altitude_tolerance = 0.1
        self.gps_available = False
        self.use_gps = False
        # Control flags
        self.takeoff_complete = False
        self.hovering = False
        self.home_position = None
        rospy.loginfo("[ArduPilot Takeoff] Controller initialized")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg
        # Store home position when first getting valid pose
        if self.home_position is None and msg.pose.position.z > -100:  # Valid altitude
            self.home_position = PoseStamped()
            self.home_position.header = msg.header
            self.home_position.pose = msg.pose
            rospy.loginfo(f"[ArduPilot Takeoff] Home position set: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")

    def extended_state_cb(self, msg):
        self.extended_state = msg

    def global_pos_cb(self, msg):
        self.global_position = msg
        # Check GPS availability
        if msg.status.status >= 0:  # GPS fix available
            self.gps_available = True
        else:
            self.gps_available = False

    def teb_status_cb(self, msg):
        self.current_teb_status = msg
        if msg.data == "HOVERING":
            self.hovering = True
        else:
            self.hovering = False

    def publish_status(self, status):
        """Publish controller status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        rospy.loginfo(f"[ArduPilot Takeoff] Status: {status}")

    def wait_for_mavros_connection(self, timeout=30):
        """Wait for MAVROS connection"""
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

    def check_gps_and_set_mode(self):
        """Check GPS availability and set appropriate flight mode"""
        if self.use_gps and self.gps_available:
            target_mode = GUIDED_MODE
            rospy.loginfo("[ArduPilot Takeoff] Using GPS-enabled GUIDED mode")
        else:
            target_mode = GUIDED_NOGPS_MODE
            rospy.loginfo("[ArduPilot Takeoff] Using GUIDED_NOGPS mode (no GPS or GPS disabled)")

        return target_mode

    def set_guided_mode(self):
        """Set vehicle to GUIDED or GUIDED_NOGPS mode based on GPS availability"""
        if not self.current_state.connected:
            rospy.logwarn("[ArduPilot Takeoff] Vehicle not connected")
            return False
        target_mode = self.check_gps_and_set_mode()
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
            takeoff_cmd = CommandTOL()
            takeoff_cmd.request.min_pitch = 0.0
            takeoff_cmd.request.yaw = 0.0
            takeoff_cmd.request.latitude = 0.0  # Use current position
            takeoff_cmd.request.longitude = 0.0
            takeoff_cmd.request.altitude = self.takeoff_altitude
            resp = self.takeoff_client.call(takeoff_cmd.request)
            if resp.success:
                rospy.loginfo(f"[ArduPilot Takeoff] Takeoff command sent to {self.takeoff_altitude}m")
                return True
            else:
                rospy.logwarn("[ArduPilot Takeoff] Takeoff command failed")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"[ArduPilot Takeoff] Takeoff service failed: {e}")
            return False

    def send_position_setpoint(self, x=None, y=None, z=None):
        """Send position setpoint for hover control"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        # Use current position if not specified
        if x is None:
            x = self.current_pose.pose.position.x
        if y is None:
            y = self.current_pose.pose.position.y
        if z is None:
            z = self.takeoff_altitude
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Maintain current orientation
        pose.pose.orientation = self.current_pose.pose.orientation
        self.local_position_pub.publish(pose)

    def check_altitude_reached(self):
        """Check if target altitude is reached"""
        current_alt = self.current_pose.pose.position.z
        target_reached = abs(current_alt - self.takeoff_altitude) < self.altitude_tolerance
        if target_reached:
            rospy.loginfo(f"[ArduPilot Takeoff] Altitude reached: {current_alt:.2f}m (target: {self.takeoff_altitude}m)")
        return target_reached

    def wait_for_altitude(self, timeout=60):
        """Wait for drone to reach target altitude"""
        rospy.loginfo(f"[ArduPilot Takeoff] Waiting for altitude {self.takeoff_altitude}m...")
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
        target_mode = self.check_gps_and_set_mode()
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
            self.send_position_setpoint()
            rate.sleep()
        self.takeoff_complete = True
        rospy.loginfo("[ArduPilot Takeoff] Takeoff sequence completed successfully!")
        return True

    def start_hover(self):
        """Maintain hover at current position"""
        if not self.takeoff_complete:
            rospy.logwarn("[ArduPilot Takeoff] Takeoff not complete, cannot hover")
            return
        rospy.loginfo("[ArduPilot Takeoff] Starting hover control...")
        self.publish_status("HOVERING")
        rate = rospy.Rate(20)  # 20Hz position control
        while not rospy.is_shutdown() and self.current_state.armed and self.hovering:
            self.send_position_setpoint()
            rate.sleep()

def main():
    try:
        controller = TakeoffHoverController()
        if controller.takeoff_sequence():
            rospy.loginfo("[takeoff] Takeoff completed successfully!")
            controller.publish_status("HOVERING")
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
