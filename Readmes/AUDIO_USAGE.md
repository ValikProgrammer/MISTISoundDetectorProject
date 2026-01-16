# Audio Package - Usage Guide

## Overview

The `audio` package provides ROS2 nodes for capturing audio from ALSA devices (like USB microphones) and streaming it over ROS2 topics for real-time analysis.

## Architecture

```
┌─────────────────┐
│  Microphone     │
│   (ALSA hw:2,0) │
└────────┬────────┘
         │
         ▼
┌─────────────────────────┐
│  audio_publisher        │
│  - Captures from ALSA   │
│  - Base64 encodes       │
│  - Publishes to topics  │
└────────┬────────────────┘
         │
         ├─► /audio_stream (raw PCM data)
         └─► /audio_metadata (format info)
         │
         ▼
┌─────────────────────────┐
│  audio_subscriber       │
│  (or your custom node)  │
│  - Decodes audio        │
│  - Analyzes/processes   │
└─────────────────────────┘
```

## Quick Start

### 1. Build the Package

```bash
cd /home/vector/Desktop/NUP/MISTI/MISTISoundDetectorProject
colcon build --packages-select audio
source install/setup.bash
```

### 2. Find Your Audio Device

On the Duckiebot:

```bash
arecord -l
```

Output example:
```
card 2: webcamproduct [webcamproduct], device 0: USB Audio [USB Audio]
```

This means your device is `hw:2,0` (card 2, device 0).

### 3. Run the Publisher

**Option A: Using ros2 run**
```bash
ros2 run audio audio_publisher --ros-args -p device:="hw:2,0"
```

**Option B: Using launch file**
```bash
ros2 launch audio audio_publisher.launch.xml device:=hw:2,0
```

**Option C: Both publisher and subscriber**
```bash
ros2 launch audio audio_pipeline.launch.xml device:=hw:2,0
```

## Parameters

The `audio_publisher` node accepts these parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device` | string | "hw:2,0" | ALSA device identifier |
| `rate` | int | 44100 | Sample rate in Hz |
| `channels` | int | 1 | Number of channels (1=mono, 2=stereo) |
| `period_size` | int | 1024 | ALSA buffer size in frames |
| `publish_rate` | float | 20.0 | How often to publish (Hz) |

### Example with Custom Parameters

```bash
ros2 run audio audio_publisher --ros-args \
  -p device:="hw:2,0" \
  -p rate:=16000 \
  -p channels:=1 \
  -p publish_rate:=30.0
```

Or with launch file:

```bash
ros2 launch audio audio_publisher.launch.xml \
  device:=hw:2,0 \
  rate:=16000 \
  publish_rate:=30.0
```

## Monitoring the Stream

### View Topics

```bash
ros2 topic list
# Should show:
# /audio_stream
# /audio_metadata
```

### Check Metadata

```bash
ros2 topic echo /audio_metadata
```

Output:
```
data: '{"rate": 44100, "channels": 1, "format": "S16_LE", "sample_width": 2}'
```

### Monitor Publication Rate

```bash
ros2 topic hz /audio_stream
```

### Subscribe and Analyze

```bash
ros2 run audio audio_subscriber
```

## Docker Usage

### Build with Audio Support

The Dockerfile should already include `python3-alsaaudio` from `requirements-apt.txt`.

### Run in Docker

**Important flags:**
- `--privileged`: Required for USB device access
- `-v /dev:/dev`: Mount devices into container
- `--network host`: For ROS2 communication

```bash
docker run --rm --privileged -v /dev:/dev --network host \
  your-image-name \
  ros2 run audio audio_publisher --ros-args -p device:="hw:2,0"
```

## Creating Your Own Audio Analysis Node

### Example: Sound Detector

Create a new node that subscribes to `/audio_stream`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import numpy as np

class SoundDetector(Node):
    def __init__(self):
        super().__init__('sound_detector')
        
        # Subscribe to audio stream
        self.sub = self.create_subscription(
            String,
            'audio_stream',
            self.audio_callback,
            10
        )
        
        # Your detection threshold
        self.threshold = 5000
    
    def audio_callback(self, msg):
        # Decode audio
        audio_data = base64.b64decode(msg.data)
        samples = np.frombuffer(audio_data, dtype=np.int16)
        
        # Analyze
        rms = np.sqrt(np.mean(samples.astype(np.float32) ** 2))
        
        if rms > self.threshold:
            self.get_logger().info(f'Sound detected! Level: {rms:.0f}')
            # Trigger your action here

def main():
    rclpy.init()
    node = SoundDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

### "Cannot open audio device"

1. Check if device exists: `arecord -l`
2. Verify permissions: `ls -l /dev/snd/`
3. In Docker, ensure `--privileged` and `-v /dev:/dev`

### "ALSA lib ... Broken configuration"

Try using `plughw:2,0` instead of `hw:2,0`:

```bash
ros2 run audio audio_publisher --ros-args -p device:="plughw:2,0"
```

### No audio data in topics

1. Check if node is running: `ros2 node list`
2. Check topics: `ros2 topic list`
3. Monitor logs: `ros2 run audio audio_publisher` (should show initialization)

### High CPU usage

Lower the `publish_rate`:

```bash
ros2 run audio audio_publisher --ros-args -p publish_rate:=10.0
```

## Performance Notes

### Data Rate Calculation

For 44100 Hz, 1 channel, 16-bit, at 20 Hz publish rate:
- Samples per packet: 44100 / 20 = 2205
- Bytes per packet: 2205 * 2 = 4410 bytes
- Data rate: 4410 * 20 = 88.2 KB/s
- With base64 encoding: ~117 KB/s

### Recommended Settings

**For sound detection:**
- rate: 16000
- channels: 1
- publish_rate: 20.0

**For high-quality recording:**
- rate: 44100
- channels: 1 or 2
- publish_rate: 30.0

**For low bandwidth:**
- rate: 8000
- channels: 1
- publish_rate: 10.0

## Next Steps

1. **Build and test**: Build the package and verify it works with your microphone
2. **Create detector**: Implement your sound detection logic in a subscriber
3. **Integrate**: Connect the detector to your robot's actions (movement, LEDs, etc.)
4. **Optimize**: Tune parameters based on your specific needs

## Related Files

- Main publisher: `src/audio/audio/audio.py`
- Example subscriber: `src/audio/audio/audio_subscriber.py`
- Package config: `src/audio/package.xml`
- Launch files: `src/audio/launch/`

