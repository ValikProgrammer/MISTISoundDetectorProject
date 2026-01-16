#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Header, Float32, String
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped

class SoundHunter(Node):
    def __init__(self):
        super().__init__('sound_hunter')
        self.vehicle_name = os.getenv('VEHICLE_NAME','duckie05')
        
        # ------------------ Subscribers ------------------
        self.create_subscription(
            Range,
            f'/{self.vehicle_name}/range',
            self.range_callback,
            1
        )

        self.create_subscription(
            Float32,
            f'/{self.vehicle_name}/frequency_volume_stream',
            self.freq_sound_callback,
            1
        )

        self.create_subscription(
            Float32,
            f'/{self.vehicle_name}/total_volume_stream',
            self.total_sound_callback,
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
        self.freq_volume = 0.0  # Target frequency volume
        self.total_volume = 0.0  # Total volume (all frequencies)
        self.sound_level = 0.0  # Effective sound level (freq if dominates, else 0)
        self.sound_history = []  # Rolling history for moving average
        self.sound_history_size = 5  # Track last 5 measurements
        self.baseline_sound_avg = 0.0  # Average sound when starting movement
        self.range = float('inf')

        self.waiting_to_scan = False      # wait before scanning
        self.wait_start_time = None
        
        self.scanning = False
        self.scan_start_time = None
        self.scan_duration = 0.0          # how long to scan (seconds)
        self.is_first_scan = True         # first scan is 360°, rescans are 90°

        self.max_sound = 0.0
        self.max_sound_time = None        # timestamp when max sound was heard

        self.rotating_to_target = False   # rotating to best direction
        self.rotate_start_time = None
        self.rotate_duration = 0.0        # how long to rotate (seconds)
        
        self.moving_forward = False       # moving toward sound
        self.move_start_time = None       # when forward movement started
        


        # ------------------ Parameters ------------------
        # Declare parameters with defaults
        self.declare_parameter('left_vel_mult', 1.0)  # left motor multiplier for calibration
        self.declare_parameter('right_vel_mult', 1.0)  # right motor multiplier for calibration
        self.declare_parameter('volume_threshold', 20.0)
        self.declare_parameter('ratio_threshold', 5.0)  # freq must be x louder than total (filters claps)
        self.declare_parameter('wait_duration', 3.0)
        self.declare_parameter('scan_360_duration', 6.0)  # seconds for full 360° scan (empirically determined)
        self.declare_parameter('scan_90_duration', 1.5)   # seconds for 90° rescan (empirically determined)
        self.declare_parameter('scan_speed', 0.3)
        self.declare_parameter('rotate_speed', 0.3)
        self.declare_parameter('forward_speed', 0.8)
        self.declare_parameter('target_range', 0.15)
        self.declare_parameter('rescan_interval', 10.0)  # rescan after 10s of movement
        self.declare_parameter('sound_decrease_threshold', 5.0)  # trigger rescan if sound drops by this amount
        
        # Get parameters
        self.left_vel_mult = float(self.get_parameter('left_vel_mult').value)  # motor calibration
        self.right_vel_mult = float(self.get_parameter('right_vel_mult').value)  # motor calibration
        self.volume_threshold = float(self.get_parameter('volume_threshold').value)  # start scanning
        self.ratio_threshold = float(self.get_parameter('ratio_threshold').value)  # freq/total ratio
        self.wait_duration = float(self.get_parameter('wait_duration').value)  # seconds to wait before scan
        self.scan_360_duration = float(self.get_parameter('scan_360_duration').value)  # 360° scan time
        self.scan_90_duration = float(self.get_parameter('scan_90_duration').value)  # 90° rescan time
        self.scan_speed = float(self.get_parameter('scan_speed').value)  # wheel speed
        self.rotate_speed = float(self.get_parameter('rotate_speed').value)  # rotation speed
        self.forward_speed = float(self.get_parameter('forward_speed').value)  # forward movement speed
        self.target_range = float(self.get_parameter('target_range').value)  # meters
        self.rescan_interval = float(self.get_parameter('rescan_interval').value)  # seconds
        self.sound_decrease_threshold = float(self.get_parameter('sound_decrease_threshold').value)

        # ------------------ Control Loop ------------------
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info(f"Sound Hunter Node started (TIME-BASED ROTATION)")
        self.get_logger().info(f"  LKW={self.left_vel_mult}, RKW={self.right_vel_mult}")
        self.get_logger().info(f"  Volume threshold: {self.volume_threshold}, Ratio threshold: {self.ratio_threshold}x")
        self.get_logger().info(f"  Wait duration: {self.wait_duration}s")
        self.get_logger().info(f"  360° scan duration: {self.scan_360_duration}s, 90° rescan duration: {self.scan_90_duration}s")
        self.get_logger().info(f"  Scan speed: {self.scan_speed}, Rotate speed: {self.rotate_speed}, Forward speed: {self.forward_speed}")
        self.get_logger().info(f"  Target range: {self.target_range}m")
        self.get_logger().info(f"  Rescan interval: {self.rescan_interval}s, Sound decrease threshold: {self.sound_decrease_threshold}")
        
    def freq_sound_callback(self, msg):
        """Callback for target frequency volume."""
        self.freq_volume = msg.data
        self._update_sound_level()
    
    def total_sound_callback(self, msg):
        """Callback for total volume."""
        self.total_volume = msg.data
        self._update_sound_level()
    
    def _update_sound_level(self):
        """Update effective sound level based on ratio detection logic."""
        # Only respond if target frequency DOMINATES (filters out claps/broadband noise)
        if self.freq_volume > self.volume_threshold and self.freq_volume > self.total_volume * self.ratio_threshold:
            # Real tone detected! (target freq is 30% louder than average)
            self.sound_level = self.freq_volume
        else:
            # Probably just a clap/broadband noise - ignore
            self.sound_level = 0.0
        
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
                self.publish_state("scanning")
                self.start_scan()
            # Rescan if: average sound is decreasing (going wrong way)
            elif current_sound_avg < self.baseline_sound_avg - self.sound_decrease_threshold:
                self.get_logger().info(
                    f"Rescan triggered: sound avg decreased from {self.baseline_sound_avg:.1f} to {current_sound_avg:.1f}"
                )
                self.stop()
                self.moving_forward = False
                self.publish_state("scanning")
                self.start_scan()
            else:
                # Continue moving toward target (sound increasing or stable = good!)
                self.move_to_target()
        

    def start_scan(self):
        """Start time-based scan: 360° first time, then 90° rescans"""
        self.scanning = True
        self.max_sound = 0.0
        self.max_sound_time = None
        self.scan_start_time = self.get_clock().now()
        
        if self.is_first_scan:
            # First scan: full 360° rotation (hardcoded duration)
            self.scan_duration = self.scan_360_duration
            self.publish_state("scanning")
            self.get_logger().info(f"Started INITIAL 360° scan | Duration: {self.scan_duration:.1f}s")
        else:
            # Rescan: 90° scan (hardcoded duration)
            self.scan_duration = self.scan_90_duration
            self.publish_state("scanning")
            self.get_logger().info(f"Started RESCAN 90° | Duration: {self.scan_duration:.1f}s")

    def scan_for_sound(self):
        """Scan by rotating right, tracking loudest sound and when it occurred"""
        # Rotate right (positive direction)
        self.run_wheels("scan", self.scan_speed, -self.scan_speed)

        # Record loudest sound and the time it occurred
        if self.sound_level > self.max_sound:
            self.max_sound = self.sound_level
            self.max_sound_time = self.get_clock().now()

        # Check if scan duration completed
        elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds * 1e-9
        
        if elapsed >= self.scan_duration:
            self.stop()
            self.scanning = False
            
            # Calculate how much to rotate back to face max sound direction
            if self.max_sound_time is not None:
                # Time from max sound until now
                time_since_max = (self.get_clock().now() - self.max_sound_time).nanoseconds * 1e-9
                # Convert time to rotation angle, then back to time for rotation
                self.rotate_duration = time_since_max
            else:
                # No sound detected, don't rotate
                self.rotate_duration = 0.0
            
            if self.is_first_scan:
                self.is_first_scan = False  # Mark first scan complete
                self.get_logger().info(
                    f"360° scan complete | Max volume: {self.max_sound:.1f} | Rotate back: {self.rotate_duration:.1f}s"
                )
            else:
                self.get_logger().info(
                    f"Rescan complete | Max volume: {self.max_sound:.1f} | Rotate back: {self.rotate_duration:.1f}s"
                )
            
            # Start rotating to target direction
            if self.rotate_duration > 0.1:  # Only rotate if significant
                self.rotating_to_target = True
                self.rotate_start_time = self.get_clock().now()
                self.publish_state("rotating")
            else:
                # Already facing the right direction, start moving
                self.moving_forward = True
                self.move_start_time = self.get_clock().now()
                self.baseline_sound_avg = self.get_sound_average()
                self.publish_state("moving_forward")
                self.get_logger().info(f"No rotation needed | Starting forward movement")

    def rotate_to_best_yaw(self):
        """Rotate back to face the direction where max sound was heard"""
        # Continue rotating in same direction (right) for the calculated duration
        self.run_wheels("rotate_back", self.scan_speed, -self.scan_speed)
        
        # Check if rotation duration completed
        elapsed = (self.get_clock().now() - self.rotate_start_time).nanoseconds * 1e-9
        
        if elapsed >= self.rotate_duration:
            # Finished rotating to target direction
            self.stop()
            self.rotating_to_target = False
            self.moving_forward = True
            self.move_start_time = self.get_clock().now()
            self.baseline_sound_avg = self.get_sound_average()
            self.publish_state("moving_forward")
            self.get_logger().info(
                f"Rotation complete | Baseline sound avg: {self.baseline_sound_avg:.1f} | Starting forward movement"
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


    def set_max_power_keeping_ratio(self, vel_left, vel_right):
        """Scale velocities so max becomes 0.85 while keeping ratio"""
        # max_vel = max(abs(vel_left), abs(vel_right))
        
        # # Avoid division by zero
        # if max_vel == 0.0:
        #     return 0.0, 0.0
        
        # # Always scale so maximum absolute value becomes 0.85
        # scale = 0.85 / max_vel
        # vel_left *= scale
        # vel_right *= scale
        
        return vel_left, vel_right

    def run_wheels(self, frame_id, vel_left, vel_right):
        msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        msg.header = header
        vel_left  = float(vel_left) * self.left_vel_mult
        vel_right = float(vel_right) * self.right_vel_mult
        
        vel_left, vel_right = self.set_max_power_keeping_ratio(vel_left, vel_right)
        
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        
        # self.get_logger().info(f"Running wheels: {msg.vel_left}({vel_left}), {msg.vel_right}({vel_right})")
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