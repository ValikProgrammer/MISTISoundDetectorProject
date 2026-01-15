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
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        # ------------------ Subscribers ------------------
        self.create_subscription(
            Range,
            f'/{self.vehicle_name}/range',
            self.range_callback,
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.vehicle_name}/volume_stream',
            self.sound_callback,
            10
        )

        self.create_subscription(
            Odometry,
            f'/{self.vehicle_name}/odom',
            self.odom_callback,
            10
        )

        # ------------------ Publishers ------------------
        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped,
            f'/{self.vehicle_name}/wheels_cmd',
            10
        )
        
        self.state_pub = self.create_publisher(
            String,
            f'/{self.vehicle_name}/robot_state',
            10
        )

        # ------------------ State ------------------
        self.sound_level = 0.0
        self.current_yaw = 0.0
        self.range = float('inf')

        self.waiting_to_scan = False      # 2-second wait before scanning
        self.wait_start_time = None
        
        self.scanning = False
        self.scan_start_time = None

        self.max_sound = 0.0
        self.best_yaw = 0.0

        self.rotating_to_target = False   # rotating to best_yaw
        self.moving_forward = False       # moving toward sound

        # ------------------ Parameters ------------------
        self.volume_threshold = 20.0      # start scanning
        self.wait_duration = 5.0          # seconds to wait before scan
        self.scan_duration = 20.0         # seconds
        self.scan_speed = 0.9             # wheel speed
        self.rotate_speed = 0.5           # rotation speed
        self.forward_speed = 0.8          # forward movement speed
        self.yaw_tolerance = 0.4          # radians (~5.7 degrees)
        self.target_range = 0.15          # 15 cm in meters

        # ------------------ Control Loop ------------------
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info("Sound Scan Node started")
        
    def sound_callback(self, msg):
        self.sound_level = msg.data

    def range_callback(self, msg):
        self.range = msg.range  # not used yet, but ready

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
            self.move_to_target()
        
        
    # def move(self): #I dont like the logic
    #     if self.current >= self.target + self.eps:
    #         self.stop()
    #         return
        
    #     if self.prev is None or self.current > self.prev + self.eps:
    #         self.run_wheels('forward', 0.2, 0.2)
    #     else:
    #         self.run_wheels('turn', 0.15, -0.15)

    #     self.prev = self.current

    # def stop(self):
    #     self.run_wheels('stop', 0.0, 0.0)

    def start_scan(self):
        self.scanning = True
        self.scan_start_time = self.get_clock().now()
        self.max_sound = 0.0
        self.best_yaw = self.current_yaw
        self.publish_state("scanning")
        self.get_logger().info("Started sound scan")

    def scan_for_sound(self):
        # Rotate in place
        self.run_wheels("scan", self.scan_speed, -self.scan_speed)

        # Record loudest sound
        if self.sound_level > self.max_sound:
            self.max_sound = self.sound_level
            self.best_yaw = self.current_yaw

        # Time check
        elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds * 1e-9
        if elapsed >= self.scan_duration:
            self.stop()
            self.scanning = False
            self.rotating_to_target = True  # start rotating to best_yaw
            self.publish_state("rotating")
            self.get_logger().info(
                f"Scan complete | Max volume: {self.max_sound:.1f} | Yaw: {self.best_yaw:.2f} rad"
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
            # Reached target yaw
            self.stop()
            self.rotating_to_target = False
            self.moving_forward = True
            self.publish_state("moving_forward")
            self.get_logger().info(f"Rotation complete | Current yaw: {self.current_yaw:.2f} rad")

    def move_to_target(self):
        if self.range > self.target_range:
            # Move forward
            self.run_wheels("forward", self.forward_speed, self.forward_speed)
        else:
            # Reached target
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

    def get_loudest_direction(self):
        """Returns yaw (radians) where sound was loudest"""
        return self.best_yaw

    def run_wheels(self, frame_id, vel_left, vel_right):
        msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        msg.header = header
        msg.vel_left = vel_left
        msg.vel_right = vel_right
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