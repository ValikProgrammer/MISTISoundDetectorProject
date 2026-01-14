#!/usr/bin/env python3
"""
Simple audio player - plays audio from /audio_stream topic.
Run on your PC to hear the Duckiebot's microphone.

Install first:
    pip3 install pyaudio

Usage:
    python3 play_audio.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import base64
import pyaudio


class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        
        # Audio parameters (will be set from metadata)
        self.rate = 44100
        self.channels = 1
        self.format = pyaudio.paInt16
        
        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        self.stream = None
        
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
        
        self.get_logger().info('🎵 Audio player started')
        self.get_logger().info('Waiting for audio stream...')
    
    def metadata_callback(self, msg):
        """Receive and parse audio metadata."""
        try:
            metadata = json.loads(msg.data)
            self.rate = metadata['rate']
            self.channels = metadata['channels']
            
            # Open audio stream for playback
            if self.stream is None:
                self.stream = self.p.open(
                    format=self.format,
                    channels=self.channels,
                    rate=self.rate,
                    output=True
                )
                self.get_logger().info(f'🎧 Audio playback started:')
                self.get_logger().info(f'   Rate: {self.rate} Hz')
                self.get_logger().info(f'   Channels: {self.channels}')
            
        except Exception as e:
            self.get_logger().error(f'Error parsing metadata: {e}')
    
    def audio_callback(self, msg):
        """Receive and play audio data."""
        try:
            # Decode base64 audio data
            audio_data = base64.b64decode(msg.data)
            
            # Play audio
            if self.stream:
                self.stream.write(audio_data)
            
        except Exception as e:
            self.get_logger().error(f'Error playing audio: {e}')
    
    def __del__(self):
        """Cleanup on exit."""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AudioPlayer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n🛑 Stopped playback')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

