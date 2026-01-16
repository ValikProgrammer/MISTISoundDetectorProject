#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Header, Float32, String
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from duckietown_msgs.msg import WheelsCmdStamped

class SoundHunter(Node):
    def __init__(self):
        super().__init__('sound_hunter')
        self.vehicle_name = os.getenv('VEHICLE_NAME','duckie05')
        self.left_vel_mult  = os.getenv('LEFT_VEL_MULT',0.8)
        self.right_vel_mult = os.getenv('RIGHT_VEL_MULT',0.6)
        
        # ------------------ Subscribers ------------------
        self.create_subscription(
            Range,
            f'/{self.vehicle_name}/range',
            self.range_callback,
            1
        )

        self.create_subscription(
            Float32,
            f'/{self.vehicle_name}/frequency_volume_stream', # chnaged from volume_stream,
            self.sound_callback,
            1
        )

        self.create_subscription(
            Odometry,
            f'/{self.vehicle_name}/odom',
            self.odom_callback,
            1
        )

        # ------------------ Publishers ------------------
        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped,
            f'/{self.vehicle_name}/wheels_cmd',
            1
        )
        
        self.state_pub = self.create_publisher(
            String,
            f'/{self.vehicle_name}/robot_state',
            1
        )

        # ------------------ State ------------------
        self.sound_level = 0.0
        self.sound_history = []  # Rolling history for moving average
        self.sound_history_size = 5  # Track last 5 measurements
        self.baseline_sound_avg = 0.0  # Average sound when starting movement
        self.current_yaw = 0.0
        self.range = float('inf')

        self.waiting_to_scan = False      # wait before scanning
        self.wait_start_time = None
        
        self.scanning = False
        self.scan_start_time = None
        self.scan_start_yaw = 0.0         # yaw when scan started
        self.scan_target_yaw = 0.0        # target yaw for scan end
        self.is_first_scan = True         # first scan is 360°, rescans are 90°

        self.max_sound = 0.0
        self.best_yaw = 0.0

        self.rotating_to_target = False   # rotating to best_yaw
        self.moving_forward = False       # moving toward sound
        self.move_start_time = None       # when forward movement started
        


        # ------------------ Parameters ------------------
        # Declare parameters with defaults
        self.declare_parameter('volume_threshold', 20.0)
        self.declare_parameter('wait_duration', 2.0)
        self.declare_parameter('scan_angle', 1.57)  # 90 degrees in radians (±45°)
        self.declare_parameter('scan_speed', 0.3)
        self.declare_parameter('rotate_speed', 0.3)
        self.declare_parameter('forward_speed', 0.8)
        self.declare_parameter('yaw_tolerance', 0.4) # 20 deg
        self.declare_parameter('target_range', 0.15)
        self.declare_parameter('rescan_interval', 10.0)  # rescan after 10s of movement
        self.declare_parameter('sound_decrease_threshold', 5.0)  # trigger rescan if sound drops by this amount
        
        # Get parameters
        self.volume_threshold = self.get_parameter('volume_threshold').value  # start scanning
        self.wait_duration = self.get_parameter('wait_duration').value  # seconds to wait before scan
        self.scan_angle = self.get_parameter('scan_angle').value  # 90° scan range (±45°)
        self.scan_speed = self.get_parameter('scan_speed').value  # wheel speed
        self.rotate_speed = self.get_parameter('rotate_speed').value  # rotation speed
        self.forward_speed = self.get_parameter('forward_speed').value  # forward movement speed
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value  # radians
        self.target_range = self.get_parameter('target_range').value  # meters
        self.rescan_interval = self.get_parameter('rescan_interval').value  # seconds
        self.sound_decrease_threshold = self.get_parameter('sound_decrease_threshold').value

        # ------------------ Control Loop ------------------
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info(f"Sound Hunter Node started")
        self.get_logger().info(f"  LKW={self.left_vel_mult}, RKW={self.right_vel_mult}")
        self.get_logger().info(f"  Volume threshold: {self.volume_threshold}")
        self.get_logger().info(f"  Wait duration: {self.wait_duration}s")
        self.get_logger().info(f"  Scan angle: {self.scan_angle:.2f} rad (±{self.scan_angle/2:.2f}), speed: {self.scan_speed}")
        self.get_logger().info(f"  Rotate speed: {self.rotate_speed}, Forward speed: {self.forward_speed}")
        self.get_logger().info(f"  Yaw tolerance: {self.yaw_tolerance} rad, Target range: {self.target_range}m")
        self.get_logger().info(f"  Rescan interval: {self.rescan_interval}s, Sound decrease threshold: {self.sound_decrease_threshold}")
        
    def sound_callback(self, msg):
        self.sound_level = msg.data
        
        # Maintain rolling history of sound levels (last 5 measurements)
        self.sound_history.append(self.sound_level)
        if len(self.sound_history) > self.sound_history_size:
            self.sound_history.pop(0)  # Remove oldest
    
    def get_sound_average(self):
        """Get average of recent sound measurements"""
        if len(self.sound_history) == 0:
            return self.sound_level
        return sum(self.sound_history) / len(self.sound_history)

    def range_callback(self, msg):
        self.range = msg.range

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def control_loop(self):
        # Detect sound and start waiting period
        if (self.sound_level > self.volume_threshold and 
            not self.waiting_to_scan and not self.scanning and 
            not self.rotating_to_target and not self.moving_forward):
            self.waiting_to_scan = True
            self.wait_start_time = self.get_clock().now()
            self.publish_state("sound_detected")
            self.get_logger().info(f"Sound detected! Waiting {self.wait_duration}s before scan... Level: {self.sound_level:.1f}")
        
        # Wait period before scanning
        if self.waiting_to_scan:
            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds * 1e-9
            if elapsed >= self.wait_duration:
                self.waiting_to_scan = False
                self.start_scan()

        # If scanning, continue scan
        if self.scanning:
            self.scan_for_sound()
        
        # After scan, rotate to best_yaw
        elif self.rotating_to_target:
            self.rotate_to_best_yaw()
        
        # After rotation, move forward until close enough
        elif self.moving_forward:
            # Check if we need to rescan
            elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds * 1e-9
            
            # Get current average sound level
            current_sound_avg = self.get_sound_average()
            
            # Rescan if: 10s elapsed AND not close to target yet
            if elapsed >= self.rescan_interval and self.range > self.target_range:
                self.get_logger().info(f"Rescan triggered: {self.rescan_interval}s elapsed, range={self.range:.2f}m")
                self.stop()
                self.moving_forward = False
                self.start_scan()
            # Rescan if: average sound is decreasing (going wrong way)
            elif current_sound_avg < self.baseline_sound_avg - self.sound_decrease_threshold:
                self.get_logger().info(
                    f"Rescan triggered: sound avg decreased from {self.baseline_sound_avg:.1f} to {current_sound_avg:.1f}"
                )
                self.stop()
                self.moving_forward = False
                self.start_scan()
            else:
                # Continue moving toward target (sound increasing or stable = good!)
                self.move_to_target()
        

    def start_scan(self):
        """Start angle-based scan: 360° first time, then 90° rescans"""
        self.scanning = True
        self.max_sound = 0.0
        self.best_yaw = self.current_yaw
        self.scan_start_yaw = self.current_yaw
        
        if self.is_first_scan:
            # First scan: full 360° rotation to find sound source
            scan_range = 2.0 * math.pi  # 360 degrees
            self.scan_target_yaw = self.scan_start_yaw + scan_range
            self.publish_state("scanning")
            self.get_logger().info(f"Started INITIAL 360° scan from {self.scan_start_yaw:.2f} rad")
        else:
            # Rescan: ±45° from current heading (90° total) to refine direction
            self.scan_start_yaw = self.current_yaw - (self.scan_angle / 2.0)
            self.scan_target_yaw = self.current_yaw + (self.scan_angle / 2.0)
            self.publish_state("scanning")
            self.get_logger().info(f"Started RESCAN: {self.scan_angle:.2f} rad (±{self.scan_angle/2:.2f})")
            self.get_logger().info(f"  From {self.scan_start_yaw:.2f} to {self.scan_target_yaw:.2f} rad")

    def scan_for_sound(self):
        """Scan by rotating right, tracking loudest direction"""
        # Rotate right (positive yaw direction)
        self.run_wheels("scan", self.scan_speed, -self.scan_speed)

        # Record loudest sound and its direction
        if self.sound_level > self.max_sound:
            self.max_sound = self.sound_level
            self.best_yaw = self.current_yaw

        # Calculate how much we've rotated from start
        angle_rotated = self.current_yaw - self.scan_start_yaw
        angle_target = self.scan_target_yaw - self.scan_start_yaw
        
        # For 360° scan, check if we've done full rotation
        if self.is_first_scan:
            # Check if rotated ~360° (2*pi radians)
            if angle_rotated >= angle_target - self.yaw_tolerance:
                self.stop()
                self.scanning = False
                self.is_first_scan = False  # Mark first scan complete
                self.rotating_to_target = True
                self.publish_state("rotating")
                self.get_logger().info(
                    f"360° scan complete | Max volume: {self.max_sound:.1f} | Best Yaw: {self.best_yaw:.2f} rad"
                )
        else:
            # For 90° rescan, check if reached target with tolerance
            angle_remaining = self.scan_target_yaw - self.current_yaw
            # Normalize to [-pi, pi]
            angle_remaining = math.atan2(math.sin(angle_remaining), math.cos(angle_remaining))
            
            if abs(angle_remaining) <= self.yaw_tolerance:  # Within tolerance
                self.stop()
                self.scanning = False
                self.rotating_to_target = True
                self.publish_state("rotating")
                self.get_logger().info(
                    f"Rescan complete | Max volume: {self.max_sound:.1f} | Best Yaw: {self.best_yaw:.2f} rad"
                )

    def rotate_to_best_yaw(self):
        # Calculate angle difference (shortest path)
        angle_diff = self.best_yaw - self.current_yaw
        # Normalize to [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        if abs(angle_diff) > self.yaw_tolerance:
            # Rotate left or right based on sign
            if angle_diff > 0:
                self.run_wheels("rotate_left", self.rotate_speed, -self.rotate_speed)
            else:
                self.run_wheels("rotate_right", -self.rotate_speed, self.rotate_speed)
        else:
            # Reached target yaw, start moving forward
            self.stop()
            self.rotating_to_target = False
            self.moving_forward = True
            self.move_start_time = self.get_clock().now()  # Start movement timer
            self.baseline_sound_avg = self.get_sound_average()  # Baseline = avg of last 5 measurements
            self.publish_state("moving_forward")
            self.get_logger().info(
                f"Rotation complete | Current yaw: {self.current_yaw:.2f} rad | "
                f"Baseline sound avg: {self.baseline_sound_avg:.1f} | Starting forward movement"
            )

    def move_to_target(self):
        if self.range > self.target_range:
            self.run_wheels("forward", self.forward_speed, self.forward_speed)
        else:
            self.stop()
            self.moving_forward = False
            self.publish_state("idle")
            self.get_logger().info(f"Target reached | Range: {self.range:.3f} m")

    # ===================================================
    # Utility methods
    # ===================================================
    
    def publish_state(self, state):
        """Publish current robot state for LED control"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def run_wheels(self, frame_id, vel_left, vel_right):
        msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        msg.header = header
        msg.vel_left  = vel_left*self.left_vel_mult
        msg.vel_right = vel_right*self.right_vel_mult
        self.get_logger().info(f"Running wheels: {msg.vel_left}({vel_left}), {msg.vel_right}({vel_right})")
        self.wheels_pub.publish(msg)

    def stop(self):
        self.run_wheels("stop", 0.0, 0.0)


def main():
    rclpy.init()
    tof = SoundHunter()
    rclpy.spin(tof)
    rclpy.shutdown()


if __name__ == '__main__':
    main()