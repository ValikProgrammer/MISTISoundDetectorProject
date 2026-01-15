#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped


class SoundHunter(Node):
    def __init__(self):
        super().__init__('sound_hunter')
        vehicle = os.getenv('VEHICLE_NAME','duckie')
        self.sound_sub = self.create_subscription(Range, f'/{vehicle}/sound_range', self.sound_callback, 10)
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{vehicle}/wheels_cmd', 10)
        self.target = float(os.getenv('TARGET_SOUND','1.0'))
        self.current = 0.0

    def sound_callback(self, msg):
        self.current = msg.range
        self.move()

    def move(self):
        if self.current < self.target:
            left = 0.2
            right = 0.2
        else:
            self.stop()

    def stop(self):
        self.run_wheels('stop_callback', 0.0, 0.0)

    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheels_pub.publish(wheel_msg)


def main():
    rclpy.init()
    tof = TofNode()
    rclpy.spin(tof)
    rclpy.shutdown()


if __name__ == '__main__':
    main()