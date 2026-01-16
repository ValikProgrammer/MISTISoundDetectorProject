#!/usr/bin/python3
"""
Volume Processor Node - Subscribes to audio_stream and publishes volume level.


"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import base64
import numpy as np
import os 


class VolumeProcessor(Node):
    def __init__(self):
        super().__init__('volume_processor')
        
        vehicle = os.getenv('VEHICLE_NAME','duckie05')
        # Declare parameters
        self.declare_parameter('smoothing_factor', 1.0)  # 1.0 = instant (no smoothing), 0.9 = 90% instant
        self.declare_parameter('normalize_to_100', True)  # Normalize volume to 0-100 scale
        self.declare_parameter('volume_scale_max', 500.0)  # Max expected RMS value for 100% (lower for laptop mic)
        
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.normalize_to_100 = self.get_parameter('normalize_to_100').value
        self.volume_scale_max = self.get_parameter('volume_scale_max').value
        
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
            f'/{vehicle}/audio_metadata',
            self.metadata_callback,
            10
        )
        
        # Subscribe to audio stream
        self.audio_sub = self.create_subscription(
            String,
            f'/{vehicle}/audio_stream',
            self.audio_callback,
            1
        )
        
        # Publisher for volume
        self.volume_pub = self.create_publisher(Float32, f'/{vehicle}/volume_stream', 1)
        
        self.get_logger().info('Volume processor started')
        self.get_logger().info(f'  Smoothing factor: {self.smoothing_factor} (1.0=instant, 0.0=max smoothing)')
        self.get_logger().info(f'  Normalize to 0-100: {self.normalize_to_100}')
        self.get_logger().info(f'  Volume scale max: {self.volume_scale_max} RMS = 100%')
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
            
            # Apply smoothing (if smoothing_factor < 1.0)
            # Formula: smoothed = alpha * new + (1 - alpha) * old
            # With alpha=0.9: 90% new, 10% old (very responsive)
            # With alpha=1.0: 100% new, 0% old (instant, no smoothing)
            self.current_volume = (
                self.smoothing_factor * raw_volume +
                (1 - self.smoothing_factor) * self.current_volume
            )
            
            # Normalize to 0-100 scale
            if self.normalize_to_100:
                # Scale based on typical max RMS (not theoretical 32767)
                # Typical loud sound: RMS ~5000
                # This makes volume more intuitive
                # min() caps the maximum at 100% (prevents going over)
                volume_normalized = min(100.0, (self.current_volume / self.volume_scale_max) * 100.0)
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
                    f'Volume: {volume_normalized:.1f}% (raw: {raw_volume:.1f}) '
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

