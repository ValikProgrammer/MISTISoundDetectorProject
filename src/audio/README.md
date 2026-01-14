# Audio Package

ROS2 package for capturing audio from ALSA devices and publishing to topics.

## Overview

This package provides:
- **Audio Publisher**: Captures audio from ALSA (e.g., USB microphone) and publishes to ROS2 topics
- **Volume Processor**: Processes audio stream and publishes volume level (0-100)
- **Audio Subscriber**: Example subscriber that receives and analyzes audio stream

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/audio_stream` | `std_msgs/String` | Raw audio data (base64 encoded PCM) |
| `/audio_metadata` | `std_msgs/String` | Audio format metadata (JSON) |
| `/volume_stream` | `std_msgs/Float32` | Volume level (0-100, normalized) |

## Nodes

### audio_publisher

Captures audio from an ALSA device and publishes to ROS2 topics.

**Parameters:**
- `device` (string, default: "hw:2,0"): ALSA device identifier
- `rate` (int, default: 44100): Sample rate in Hz
- `channels` (int, default: 1): Number of audio channels (1=mono, 2=stereo)
- `period_size` (int, default: 1024): ALSA buffer size in frames
- `publish_rate` (float, default: 20.0): Publishing frequency in Hz

**Example usage:**
```bash
# Run with defaults
ros2 run audio audio_publisher

# Run with custom parameters
ros2 run audio audio_publisher --ros-args \
  -p device:="hw:2,0" \
  -p rate:=44100 \
  -p channels:=1 \
  -p publish_rate:=20.0
```

### volume_processor

Processes raw audio stream and publishes volume level (loudness).

**Parameters:**
- `smoothing_factor` (float, default: 0.3): Volume smoothing (0-1, lower=smoother)
- `normalize_to_100` (bool, default: true): Normalize volume to 0-100 scale

**Example usage:**
```bash
# Run with defaults
ros2 run audio volume_processor

# Run with custom smoothing
ros2 run audio volume_processor --ros-args -p smoothing_factor:=0.5

# Run complete pipeline (publisher + volume processor)
ros2 launch audio audio_with_volume.launch.xml device:=hw:2,0
```

**See also:** `VOLUME_PROCESSOR.md` for detailed documentation.

### audio_subscriber

Example subscriber that receives audio stream and performs basic analysis.

**Example usage:**
```bash
ros2 run audio audio_subscriber
```

## Building

From the workspace root:

```bash
cd /home/vector/Desktop/NUP/MISTI/MISTISoundDetectorProject
colcon build --packages-select audio
source install/setup.bash
```

## Docker Usage

Make sure the Dockerfile includes `python3-alsaaudio` (already configured in requirements-apt.txt).

Run with privileged mode to access audio devices:

```bash
docker run --rm --privileged -v /dev:/dev \
  --network host \
  your-image-name \
  ros2 run audio audio_publisher --ros-args -p device:="hw:2,0"
```

## Audio Format

- **Format**: PCM S16_LE (16-bit signed little-endian)
- **Encoding**: Base64 (for safe ROS2 string transmission)
- **Default rate**: 44100 Hz
- **Default channels**: 1 (mono)

## Finding Your Audio Device

On the Duckiebot, list available capture devices:

```bash
arecord -l
```

Look for your USB microphone (e.g., "webcamproduct"). If it shows as card 2, device 0, use `hw:2,0`.

## Example: Testing the Audio Pipeline

### Option 1: All-in-one launch

```bash
ros2 launch audio audio_with_volume.launch.xml device:=hw:2,0
```

### Option 2: Manual setup

Terminal 1 (Publisher):
```bash
ros2 run audio audio_publisher --ros-args -p device:="hw:2,0"
```

Terminal 2 (Volume Processor):
```bash
ros2 run audio volume_processor
```

Terminal 3 (Monitor volume):
```bash
# See volume in real-time
ros2 topic echo /volume_stream
```

Terminal 4 (Monitor topics):
```bash
# See topic list
ros2 topic list

# Monitor audio metadata
ros2 topic echo /audio_metadata

# Monitor publication rate
ros2 topic hz /audio_stream
```

## Analyzing Audio Data

The subscriber receives audio as numpy arrays. Modify `audio_subscriber.py`'s `process_audio()` function for custom analysis:

```python
def process_audio(self, samples):
    # samples is a numpy array of int16
    # Your analysis code here:
    # - FFT for frequency analysis
    # - Sound detection algorithms
    # - Speech recognition
    # - etc.
    pass
```

## Dependencies

- `rclpy`: ROS2 Python client library
- `std_msgs`: Standard ROS2 messages
- `alsaaudio`: Python ALSA audio interface (apt: python3-alsaaudio)
- `numpy`: Numerical operations on audio data

