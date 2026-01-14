#!/usr/bin/python3
"""
Volume Processor Node - Subscribes to audio_stream and publishes volume level.

Author: Eugen
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import base64
import numpy as np


class VolumeProcessor(Node):
    def __init__(self):
        super().__init__('volume_processor')
        
        # Declare parameters
        self.declare_parameter('smoothing_factor', 0.3)  # For smoothing volume changes
        self.declare_parameter('normalize_to_100', True)  # Normalize volume to 0-100 scale
        
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.normalize_to_100 = self.get_parameter('normalize_to_100').value
        
        # Audio parameters (will be set from metadata)
        self.rate = 44100
        self.channels = 1
        self.sample_width = 2
        self.metadata_received = False
        
        # Volume tracking
        self.current_volume = 0.0
        self.max_volume_seen = 1.0  # For dynamic normalization
        
        # Subscribe to metadata
        self.metadata_sub = self.create_subscription(
            String,
            'audio_metadata',
            self.metadata_callback,
            10
        )
        
        # Subscribe to audio stream
        self.audio_sub = self.create_subscription(
            String,
            'audio_stream',
            self.audio_callback,
            10
        )
        
        # Publisher for volume
        self.volume_pub = self.create_publisher(Float32, 'volume_stream', 10)
        
        self.get_logger().info('Volume processor started')
        self.get_logger().info(f'  Smoothing factor: {self.smoothing_factor}')
        self.get_logger().info(f'  Normalize to 0-100: {self.normalize_to_100}')
        self.get_logger().info('Waiting for audio stream...')
        
        # Statistics
        self.packets_processed = 0
    
    def metadata_callback(self, msg):
        """Receive and parse audio metadata."""
        try:
            metadata = json.loads(msg.data)
            self.rate = metadata['rate']
            self.channels = metadata['channels']
            self.sample_width = metadata['sample_width']
            
            if not self.metadata_received:
                self.get_logger().info(f'Received audio metadata:')
                self.get_logger().info(f'  Rate: {self.rate} Hz')
                self.get_logger().info(f'  Channels: {self.channels}')
                self.get_logger().info(f'  Sample width: {self.sample_width} bytes')
                self.metadata_received = True
                
        except Exception as e:
            self.get_logger().error(f'Error parsing metadata: {e}')
    
    def calculate_volume(self, samples: np.ndarray) -> float:
        """
        Calculate volume (RMS - Root Mean Square) from audio samples.
        
        Args:
            samples: numpy array of int16 audio samples
            
        Returns:
            volume: RMS value
        """
        if len(samples) == 0:
            return 0.0
        
        # Calculate RMS (Root Mean Square)
        rms = np.sqrt(np.mean(samples.astype(np.float32) ** 2))
        
        return float(rms)
    
    def audio_callback(self, msg):
        """Process audio data and publish volume."""
        try:
            # Decode base64 audio data
            audio_data = base64.b64decode(msg.data)
            
            # Convert to numpy array
            samples = np.frombuffer(audio_data, dtype=np.int16)
            
            # Calculate raw volume (RMS)
            raw_volume = self.calculate_volume(samples)
            
            # Apply smoothing to reduce jitter
            # Formula: smoothed = alpha * new + (1 - alpha) * old
            self.current_volume = (
                self.smoothing_factor * raw_volume +
                (1 - self.smoothing_factor) * self.current_volume
            )
            
            # Normalize if requested
            if self.normalize_to_100:
                # For 16-bit audio, max theoretical value is 32767
                # Normalize to 0-100 scale
                volume_normalized = min(100.0, (self.current_volume / 327.67))
            else:
                volume_normalized = self.current_volume
            
            # Publish volume
            volume_msg = Float32()
            volume_msg.data = volume_normalized
            self.volume_pub.publish(volume_msg)
            
            self.packets_processed += 1
            
            # Log every 10 packets (~1 second at 10Hz)
            if self.packets_processed % 10 == 0:
                self.get_logger().info(
                    f'🔊 Volume: {volume_normalized:.1f}% '
                    f'(packet #{self.packets_processed})'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VolumeProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

