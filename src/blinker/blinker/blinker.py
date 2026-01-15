#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA, String


class Blinker(Node):
    def __init__(self):
        super().__init__('blinker')
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        
        # Publisher for LED patterns
        self.publisher = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
        
        # Subscribe to robot state
        self.create_subscription(
            String,
            f'/{self.vehicle_name}/robot_state',
            self.state_callback,
            10
        )
        
        self.robot_state = "idle"  # idle, sound_detected, scanning, rotating, moving_forward
        self.timer = self.create_timer(0.2, self.publish_pattern)  # 5 Hz for smooth updates

    def state_callback(self, msg):
        """Update robot state based on movement node"""
        self.robot_state = msg.data
        self.get_logger().info(f"State changed: {self.robot_state}")

    def publish_pattern(self):
        """Publish LED pattern based on robot state"""
        msg = LEDPattern()

        # Choose color based on state
        if self.robot_state == "sound_detected":
            # RED - sound detected
            pattern = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        elif self.robot_state == "scanning":
            # YELLOW - scanning for maximum sound
            pattern = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        elif self.robot_state == "rotating":
            # ORANGE - rotating to target
            pattern = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        elif self.robot_state == "moving_forward":
            # GREEN - moving toward target
            pattern = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        else:
            # BLUE - idle
            pattern = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)

        # 5 LEDs to fill
        msg.rgb_vals = [pattern] * 5
        self.publisher.publish(msg)


def main():
    rclpy.init()
    blinker = Blinker()
    rclpy.spin(blinker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
