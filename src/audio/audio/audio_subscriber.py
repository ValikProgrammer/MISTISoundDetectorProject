#!/usr/bin/python3
"""
Audio Subscriber Node - Subscribes to audio stream and processes it.
This is an example subscriber for testing and analysis.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import base64
import numpy as np


class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')
        
        # Audio parameters (will be set from metadata)
        self.rate = 44100
        self.channels = 1
        self.sample_width = 2
        
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
        
        self.get_logger().info('Audio subscriber started')
        self.get_logger().info('Waiting for audio metadata...')
        
        # Statistics
        self.packets_received = 0
        self.bytes_received = 0
    
    def metadata_callback(self, msg):
        """Receive and parse audio metadata."""
        try:
            metadata = json.loads(msg.data)
            self.rate = metadata['rate']
            self.channels = metadata['channels']
            self.sample_width = metadata['sample_width']
            
            self.get_logger().info(f'Received audio metadata:')
            self.get_logger().info(f'  Rate: {self.rate} Hz')
            self.get_logger().info(f'  Channels: {self.channels}')
            self.get_logger().info(f'  Sample width: {self.sample_width} bytes')
            
        except Exception as e:
            self.get_logger().error(f'Error parsing metadata: {e}')
    
    def audio_callback(self, msg):
        """Receive and process audio data."""
        try:
            # Decode base64 audio data
            audio_data = base64.b64decode(msg.data)
            
            # Convert to numpy array for processing
            samples = np.frombuffer(audio_data, dtype=np.int16)
            
            self.packets_received += 1
            self.bytes_received += len(audio_data)
            
            # Example analysis: calculate RMS (audio level)
            if len(samples) > 0:
                rms = np.sqrt(np.mean(samples.astype(np.float32) ** 2))
                level = min(100, rms / 327.67)  # Normalize to 0-100
                
                # Log every 50th packet
                if self.packets_received % 50 == 0:
                    self.get_logger().info(
                        f'Received {self.packets_received} packets, '
                        f'{self.bytes_received/1024:.1f} KB, '
                        f'audio level: {level:.1f}%'
                    )
            
            # TODO: Add your audio analysis here
            # self.process_audio(samples)
            
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')
    
    def process_audio(self, samples):
        """
        Process audio samples - implement your analysis here.
        
        Args:
            samples: numpy array of int16 audio samples
        """
        # Example: Detect loud sounds
        rms = np.sqrt(np.mean(samples.astype(np.float32) ** 2))
        threshold = 5000  # Adjust based on your needs
        
        if rms > threshold:
            self.get_logger().info(f'Loud sound detected! RMS: {rms:.0f}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AudioSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

