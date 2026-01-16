#!/usr/bin/python3
"""
Frequency-Specific Volume Processor
Detects volume of a specific frequency range (e.g., 500Hz tone from phone)
"""
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import numpy as np
import base64


class FrequencyVolumeProcessor(Node):
    def __init__(self):
        super().__init__('frequency_volume_processor')
        vehicle = os.getenv('VEHICLE_NAME', 'duckie05')
        
        # Declare parameters
        self.declare_parameter('target_frequency', 500.0)  # Hz (e.g., 500Hz tone)
        self.declare_parameter('frequency_bandwidth', 50.0)  # Hz (±50Hz = 450-550Hz)
        self.declare_parameter('smoothing_factor', 1.0)  # 0.9  (1 = instant)
        self.declare_parameter('normalize_to_100', True)  # 0-100 scale
        self.declare_parameter('volume_scale_max', 500.0)  # Max total RMS for 100%
        self.declare_parameter('freq_volume_scale_max', 5000.0)  # Max frequency RMS for 100% (FFT magnitude is higher)
        self.declare_parameter('log_interval', 10)  # Log every N packets
        
        # Get parameters
        self.target_freq = self.get_parameter('target_frequency').value
        self.freq_bandwidth = self.get_parameter('frequency_bandwidth').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.normalize_to_100 = self.get_parameter('normalize_to_100').value
        self.volume_scale_max = self.get_parameter('volume_scale_max').value
        self.freq_volume_scale_max = self.get_parameter('freq_volume_scale_max').value
        self.log_interval = self.get_parameter('log_interval').value
        
        # Calculate frequency range
        self.freq_min = self.target_freq - (self.freq_bandwidth / 2.0)
        self.freq_max = self.target_freq + (self.freq_bandwidth / 2.0)
        
        # Audio parameters (set from metadata)
        self.rate = 16000
        self.channels = 1
        self.sample_width = 2
        self.metadata_received = False
        
        # Volume tracking
        self.current_volume = 0.0
        self.current_total_volume = 0.0  # Full spectrum volume for comparison
        
        # Subscribe to audio metadata
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
        
        # Publishers
        self.freq_volume_pub = self.create_publisher(
            Float32, 
            f'/{vehicle}/frequency_volume_stream', 
            1
        )
        self.total_volume_pub = self.create_publisher(
            Float32,
            f'/{vehicle}/total_volume_stream',
            1
        )
        
        self.get_logger().info('Frequency Volume Processor started')
        self.get_logger().info(f'  Target frequency: {self.target_freq} Hz')
        self.get_logger().info(f'  Frequency range: {self.freq_min:.1f} - {self.freq_max:.1f} Hz')
        self.get_logger().info(f'  Smoothing factor: {self.smoothing_factor}')
        self.get_logger().info(f'  Normalize to 0-100: {self.normalize_to_100}')
        self.get_logger().info(f'  Frequency scale max: {self.freq_volume_scale_max} (for target freq)')
        self.get_logger().info(f'  Total volume scale max: {self.volume_scale_max} (for all freqs)')
        self.get_logger().info(f'  Log interval: every {self.log_interval} packets')
        self.get_logger().info('Waiting for audio stream...')
        
        # Statistics
        self.packets_processed = 0
    
    def metadata_callback(self, msg):
        """Receive and parse audio metadata."""
        try:
            import json
            metadata = json.loads(msg.data)
            self.rate = metadata.get('rate', 16000)
            self.channels = metadata.get('channels', 1)
            self.sample_width = metadata.get('sample_width', 2)
            
            if not self.metadata_received:
                self.get_logger().info(f'Received metadata: {self.rate}Hz, {self.channels}ch')
                self.metadata_received = True
                
        except Exception as e:
            self.get_logger().error(f'Error parsing metadata: {e}')
    
    def calculate_frequency_volume(self, samples: np.ndarray) -> tuple:
        """
        Calculate volume of specific frequency range using FFT.
        
        Returns:
            (freq_volume, total_volume, peak_freq, peak_magnitude): 
                Volume at target frequency, total volume, loudest frequency, and its magnitude
        """
        if len(samples) == 0:
            return 0.0, 0.0, 0.0, 0.0
        
        # Calculate total RMS (full spectrum)
        total_rms = np.sqrt(np.mean(samples.astype(np.float32) ** 2))
        
        # Perform FFT to get frequency components
        fft_result = np.fft.rfft(samples)
        fft_magnitude = np.abs(fft_result)
        
        # Get frequency bins
        freqs = np.fft.rfftfreq(len(samples), 1.0 / self.rate)
        
        # Find frequency with highest magnitude (excluding DC component at 0Hz)
        if len(fft_magnitude) > 1:
            peak_idx = np.argmax(fft_magnitude[1:]) + 1  # Skip DC (0Hz)
            peak_freq = freqs[peak_idx]
            peak_magnitude = fft_magnitude[peak_idx]
        else:
            peak_freq = 0.0
            peak_magnitude = 0.0
        
        # Find indices for target frequency range
        freq_mask = (freqs >= self.freq_min) & (freqs <= self.freq_max)
        
        # Calculate RMS of target frequency range
        if np.any(freq_mask):
            freq_magnitudes = fft_magnitude[freq_mask]
            freq_rms = np.sqrt(np.mean(freq_magnitudes ** 2))
        else:
            freq_rms = 0.0
        
        return float(freq_rms), float(total_rms), float(peak_freq), float(peak_magnitude)
    
    def audio_callback(self, msg):
        """Process audio data and publish frequency-specific volume."""
        try:
            # Decode base64 audio data
            audio_data = base64.b64decode(msg.data)
            
            # Convert to numpy array
            samples = np.frombuffer(audio_data, dtype=np.int16)
            
            # Calculate frequency-specific and total volume, plus peak frequency
            raw_freq_volume, raw_total_volume, peak_freq, peak_magnitude = self.calculate_frequency_volume(samples)
            
            # Apply smoothing
            self.current_volume = (
                self.smoothing_factor * raw_freq_volume +
                (1 - self.smoothing_factor) * self.current_volume
            )
            
            self.current_total_volume = (
                self.smoothing_factor * raw_total_volume +
                (1 - self.smoothing_factor) * self.current_total_volume
            )
            
            # Normalize to 0-100 scale (use separate scales for frequency and total)
            if self.normalize_to_100:
                freq_volume_normalized = min(
                    100.0, 
                    (self.current_volume / self.freq_volume_scale_max) * 100.0
                )
                total_volume_normalized = min(
                    100.0,
                    (self.current_total_volume / self.volume_scale_max) * 100.0
                )
            else:
                freq_volume_normalized = self.current_volume
                total_volume_normalized = self.current_total_volume
            
            # Publish frequency-specific volume
            freq_msg = Float32()
            freq_msg.data = freq_volume_normalized
            self.freq_volume_pub.publish(freq_msg)
            
            # Publish total volume
            total_msg = Float32()
            total_msg.data = total_volume_normalized
            self.total_volume_pub.publish(total_msg)
            
            self.packets_processed += 1
            
            # Log every N packets (configurable via log_interval parameter)
            if self.packets_processed % self.log_interval == 0:
                ratio = (freq_volume_normalized / total_volume_normalized * 100) if total_volume_normalized > 0 else 0
                self.get_logger().info(
                    f'Target {self.target_freq:.0f}Hz: {freq_volume_normalized:.1f}% | '
                    f'Total: {total_volume_normalized:.1f}% | '
                    f'Ratio: {ratio:.0f}% | '
                    f'🎵 LOUDEST: {peak_freq:.0f}Hz ({peak_magnitude:.0f})'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FrequencyVolumeProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

