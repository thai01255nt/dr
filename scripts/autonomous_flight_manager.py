#!/usr/bin/env python3

import rospy
import subprocess
import time
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import signal
import sys

class AutonomousFlightManager:
    def __init__(self):
        rospy.init_node('autonomous_flight_manager', anonymous=True)
        # State variables
        self.drone_state = State()
        self.map_received = False
        self.cartographer_launched = False
        self.movebase_launched = False
        self.rviz_launched = False
        self.takeoff_complete = False
        # Launch processes
        self.cartographer_process = None
        self.movebase_process = None
        self.rviz_process = None
        self.teb_controller_process = None
        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        # Current position
        self.current_pose = PoseStamped()
        rospy.loginfo("Autonomous Flight Manager initialized")
        # Setup signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

    def state_cb(self, msg):
        self.drone_state = msg

    def map_cb(self, msg):
        if not self.map_received:
            rospy.loginfo("Map received from Cartographer!")
            self.map_received = True

    def pose_cb(self, msg):
        self.current_pose = msg

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        rospy.loginfo("Shutting down Autonomous Flight Manager...")
        self.shutdown_all_processes()
        sys.exit(0)

    def execute_takeoff_and_hover(self):
        """Execute takeoff using takeoff_controller"""
        try:
            # Launch takeoff controller
            rospy.loginfo("Starting takeoff sequence...")
            takeoff_process = subprocess.Popen(['rosrun', 'cart_teb_test', 'takeoff_and_hover.py'])
            # Monitor takeoff progress
            timeout = 60  # 60 seconds timeout for takeoff
            start_time = time.time()
            while takeoff_process.poll() is None and time.time() - start_time < timeout:
                # Check if altitude reached (1m ± 10cm)
                if hasattr(self.current_pose.pose.position, 'z'):
                    current_alt = self.current_pose.pose.position.z
                    if 0.9 <= current_alt <= 1.1 and self.drone_state.mode == "OFFBOARD":
                        self.takeoff_complete = True
                        rospy.loginfo(f"Takeoff complete at altitude: {current_alt:.2f}m")
                        # Give some time to stabilize
                        time.sleep(5)
                        return True
                time.sleep(0.5)
            if takeoff_process.poll() is None:
                takeoff_process.terminate()
                rospy.logerr("Takeoff timeout!")
                return False
            return self.takeoff_complete
        except Exception as e:
            rospy.logerr(f"Takeoff failed: {e}")
            return False

    def launch_cartographer(self):
        """Launch Cartographer SLAM"""
        if self.cartographer_launched:
            return True
        try:
            rospy.loginfo("Launching Cartographer...")
            self.cartographer_process = subprocess.Popen(['roslaunch', 'cart_teb_test', 'cartographer.launch'])
            time.sleep(5)
            if self.cartographer_process.poll() is None:
                self.cartographer_launched = True
                rospy.loginfo("Cartographer launched successfully")
                return True
            else:
                rospy.logerr("Failed to launch Cartographer")
                return False
        except Exception as e:
            rospy.logerr(f"Failed to launch Cartographer: {e}")
            return False

    def wait_for_map(self, timeout=30):
        """Wait for map to be published by Cartographer"""
        rospy.loginfo("Waiting for map from Cartographer...")
        start_time = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.map_received:
            if time.time() - start_time > timeout:
                rospy.logerr("Timeout waiting for map")
                return False
            rate.sleep()
        rospy.loginfo("Map received!")
        return True

    def launch_movebase_and_rviz(self):
        """Launch move_base and rviz"""
        try:
            # Launch move_base
            if not self.movebase_launched:
                rospy.loginfo("Launching move_base...")
                self.movebase_process = subprocess.Popen(['roslaunch', 'cart_teb_test', 'move_base.launch'])
                time.sleep(3)
                if self.movebase_process.poll() is None:
                    self.movebase_launched = True
                    rospy.loginfo("move_base launched successfully")
                else:
                    rospy.logerr("Failed to launch move_base")
                    return False
            # Launch rviz
            if not self.rviz_launched:
                rospy.loginfo("Launching rviz...")
                self.rviz_process = subprocess.Popen(['roslaunch', 'cart_teb_test', 'rviz.launch'])
                time.sleep(2)
                if self.rviz_process.poll() is None:
                    self.rviz_launched = True
                    rospy.loginfo("rviz launched successfully")
                else:
                    rospy.logerr("Failed to launch rviz")
                    return False
            return True
        except Exception as e:
            rospy.logerr(f"Failed to launch move_base/rviz: {e}")
            return False

    def launch_teb_controller(self):
        """Launch TEB hover controller"""
        try:
            rospy.loginfo("Launching TEB hover controller...")
            self.teb_controller_process = subprocess.Popen(['rosrun', 'cart_teb_test', 'teb_controller.py'])
            time.sleep(2)
            if self.teb_controller_process.poll() is None:
                rospy.loginfo("TEB hover controller launched successfully")
                return True
            else:
                rospy.logerr("Failed to launch TEB hover controller")
                return False
        except Exception as e:
            rospy.logerr(f"Failed to launch TEB controller: {e}")
            return False
    def launch_simple_goal_forwarder(self):
        try:
            rospy.loginfo("Launching simple goal forwarder...")
            self.teb_controller_process = subprocess.Popen(['rosrun', 'cart_teb_test', 'simple_goal_forwarder.py'])
            time.sleep(2)
            if self.teb_controller_process.poll() is None:
                rospy.loginfo("Simple goal forwarder launched successfully")
                return True
            else:
                rospy.logerr("Failed to launch simple goal forwarder")
                return False
        except Exception as e:
            rospy.logerr(f"Failed to launch simple goal forwarder: {e}")
            return False


    def shutdown_all_processes(self):
        """Shutdown all launched processes"""
        rospy.loginfo("Shutting down all processes...")
        processes = [
            (self.teb_controller_process, "TEB Controller"),
            (self.rviz_process, "RViz"),
            (self.movebase_process, "move_base"),
            (self.cartographer_process, "Cartographer")
        ]
        for process, name in processes:
            if process and process.poll() is None:
                rospy.loginfo(f"Terminating {name}...")
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    rospy.logwarn(f"Force killing {name}...")
                    process.kill()

    def run_sequence(self):
        """Run the complete autonomous flight sequence"""
        try:
            rospy.loginfo("=== Starting Autonomous Flight Sequence ===")
            # Step 1: Takeoff to 1m
            if not self.execute_takeoff_and_hover():
                rospy.logerr("Takeoff failed! Aborting sequence.")
                return False
            # Step 2: Launch Cartographer
            if not self.launch_cartographer():
                rospy.logerr("Cartographer launch failed! Aborting sequence.")
                return False
            # Step 3: Wait for map
            if not self.wait_for_map():
                rospy.logerr("Map generation failed! Aborting sequence.")
                return False
            # Step 4: Launch move_base and rviz
            if not self.launch_movebase_and_rviz():
                rospy.logerr("Navigation stack launch failed! Aborting sequence.")
                return False
            # Step 5: Launch TEB controller
            if not self.launch_teb_controller():
                rospy.logerr("TEB controller launch failed! Aborting sequence.")
                return False
            # Step 6: System ready
            rospy.loginfo("=== System Ready! You can now set goals in RViz ===")
            # Keep the manager alive
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                # Monitor system health
                if (self.cartographer_process and self.cartographer_process.poll() is not None):
                    rospy.logerr("Cartographer died!")
                    break
                if (self.movebase_process and self.movebase_process.poll() is not None):
                    rospy.logerr("move_base died!")
                    break
                if (self.teb_controller_process and self.teb_controller_process.poll() is not None):
                    rospy.logerr("TEB controller died!")
                    break
                rate.sleep()
            return True
        except rospy.ROSInterruptException:
            rospy.loginfo("Sequence interrupted by user")
            return False
        except Exception as e:
            rospy.logerr(f"Sequence failed: {e}")
            return False
        finally:
            self.shutdown_all_processes()

def main():
    try:
        manager = AutonomousFlightManager()
        rospy.loginfo("Autonomous Flight Manager started")
        rospy.loginfo("Make sure PX4 and MAVROS are running!")
        # Wait a bit for connections
        time.sleep(2)
        # Run the sequence
        success = manager.run_sequence()
        if success:
            rospy.loginfo("Autonomous flight sequence completed successfully!")
        else:
            rospy.logerr("Autonomous flight sequence failed!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Autonomous flight manager interrupted")
    except Exception as e:
        rospy.logerr(f"Autonomous flight manager error: {e}")

if __name__ == '__main__':
    main()

