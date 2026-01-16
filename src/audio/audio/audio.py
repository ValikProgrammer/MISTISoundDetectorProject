#!/usr/bin/python3
"""
Audio Publisher Node - Captures audio from ALSA and publishes to ROS2 topic.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import alsaaudio
import struct
import json
import os 

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')
        vehicle = os.getenv('VEHICLE_NAME','duckie05')
        # Declare parameters with defaults
        self.declare_parameter('device', 'plughw:2,0')
        self.declare_parameter('rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('period_size', 1024)
        self.declare_parameter('publish_rate', 20.0)  # Hz - 20Hz = 50ms updates (responsive!)
        
        # Get parameters
        device = self.get_parameter('device').value
        rate = self.get_parameter('rate').value
        channels = self.get_parameter('channels').value
        period_size = self.get_parameter('period_size').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Configure ALSA audio capture
        try:
            self.inp = alsaaudio.PCM(
                type=alsaaudio.PCM_CAPTURE,
                mode=alsaaudio.PCM_NORMAL,
                device=device,
            )
            
            self.actual_channels = self.inp.setchannels(channels)
            self.actual_rate = self.inp.setrate(rate)
            self.inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
            self.inp.setperiodsize(period_size)
            
            self.get_logger().info(f'Audio capture configured:')
            self.get_logger().info(f'  Device: {device}')
            self.get_logger().info(f'  Rate: {self.actual_rate} Hz (requested {rate})')
            self.get_logger().info(f'  Channels: {self.actual_channels} (requested {channels})')
            self.get_logger().info(f'  Period size: {period_size} frames')
            self.get_logger().info(f'  Format: S16_LE (16-bit signed little-endian)')
            
        except alsaaudio.ALSAAudioError as e:
            self.get_logger().error(f'Failed to initialize ALSA: {e}')
            raise
        
        # Create publisher for raw audio data
        self.audio_pub = self.create_publisher(String, f'/{vehicle}/audio_stream', 10)
        
        # Create publisher for audio metadata (rate, channels, etc)
        self.metadata_pub = self.create_publisher(String, f'/{vehicle}/audio_metadata', 10)
        
        # Publish metadata at startup
        self.publish_metadata()
        
        # Create timer to read and publish audio at specified rate
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.capture_and_publish)
        
        self.get_logger().info(f'Audio publisher started, publishing at {publish_rate} Hz')
        
        # Statistics
        self.packets_published = 0
        self.bytes_published = 0
    
    def publish_metadata(self):
        """Publish audio metadata to help subscribers decode the stream."""
        metadata = {
            'rate': self.actual_rate,
            'channels': self.actual_channels,
            'format': 'S16_LE',  # 16-bit signed little-endian
            'sample_width': 2,  # bytes per sample
        }
        
        msg = String()
        msg.data = json.dumps(metadata)
        self.metadata_pub.publish(msg)
        self.get_logger().info(f'Published audio metadata: {metadata}')
    
    def capture_and_publish(self):
        """Capture audio chunk and publish to topic."""
        try:
            # Publish metadata every 20 packets so new subscribers can sync
            if self.packets_published > 0 and self.packets_published % 20 == 0:
                self.publish_metadata()
            
            # Read audio data from ALSA
            length, data = self.inp.read()
            
            if length > 0 and data:
                # Create message with audio data
                # We encode as base64 to safely send binary data as string
                import base64
                
                # Ensure data is bytes type to avoid PY_SSIZE_T_CLEAN error
                if not isinstance(data, bytes):
                    data = bytes(data)
                
                encoded_data = base64.b64encode(data).decode('ascii')
                
                msg = String()
                msg.data = encoded_data
                
                # Publish
                self.audio_pub.publish(msg)
                
                self.packets_published += 1
                self.bytes_published += len(data)
                
                # Log stats every 10 packets (~1 second at 10Hz)
                if self.packets_published % 10 == 0:
                    self.get_logger().info(
                        f'📦 Published {self.packets_published} packets, '
                        f'{self.bytes_published/1024:.1f} KB '
                        f'(~{self.packets_published // 10} seconds of audio)'
                    )
                    
        except alsaaudio.ALSAAudioError as e:
            self.get_logger().error(f'ALSA read error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error capturing audio: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AudioPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
