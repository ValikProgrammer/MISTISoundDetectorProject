# Volume Processor Node

## Overview

The **volume_processor** node subscribes to the raw `audio_stream` and publishes the volume level (loudness) to `volume_stream`.

This is useful for sound detection, audio level monitoring, and triggering actions based on loudness.

## What It Does

```
/audio_stream → volume_processor → /volume_stream
(raw PCM)       (calculates RMS)   (Float32: 0-100)
```

- **Input**: `/audio_stream` (base64-encoded PCM audio)
- **Output**: `/volume_stream` (Float32, normalized to 0-100)
- **Method**: RMS (Root Mean Square) with smoothing

## Usage

### Run Standalone

```bash
# Terminal 1: Start audio publisher
ros2 run audio audio_publisher --ros-args -p device:="hw:2,0"

# Terminal 2: Start volume processor
ros2 run audio volume_processor
```

### Run with Launch File

```bash
# Start both publisher and volume processor
ros2 launch audio audio_with_volume.launch.xml device:=hw:2,0
```

### Monitor Volume

```bash
# See volume values
ros2 topic echo /volume_stream

# Monitor update rate
ros2 topic hz /volume_stream
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `smoothing_factor` | float | 0.3 | Smoothing (0=very smooth, 1=no smoothing) |
| `normalize_to_100` | bool | true | Normalize volume to 0-100 scale |

### Smoothing Factor

Controls how quickly volume responds to changes:
- **0.1**: Very smooth, slow response
- **0.3**: Balanced (default)
- **0.7**: Fast response, more jittery
- **1.0**: No smoothing, raw values

Example:
```bash
ros2 run audio volume_processor --ros-args -p smoothing_factor:=0.5
```

## Volume Scale

With `normalize_to_100:=true` (default):
- **0-10**: Very quiet / silence
- **10-30**: Normal speaking
- **30-60**: Loud voice / music
- **60-100**: Very loud sounds

Raw RMS values (if `normalize_to_100:=false`):
- 0-32767 (16-bit audio range)

## Example: Detect Loud Sounds

Create a node that reacts to volume:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SoundDetector(Node):
    def __init__(self):
        super().__init__('sound_detector')
        
        self.threshold = 40.0  # Trigger at volume > 40
        
        self.sub = self.create_subscription(
            Float32,
            'volume_stream',
            self.volume_callback,
            10
        )
    
    def volume_callback(self, msg):
        volume = msg.data
        
        if volume > self.threshold:
            self.get_logger().info(f'LOUD SOUND! Volume: {volume:.1f}')
            # Your action here: move robot, flash LEDs, etc.

def main():
    rclpy.init()
    node = SoundDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Architecture

The complete audio pipeline:

```
┌──────────────┐
│  Microphone  │
└──────┬───────┘
       │
       ▼
┌─────────────────┐
│ audio_publisher │
└────────┬────────┘
         │
         ├─► /audio_stream (raw PCM)
         └─► /audio_metadata (format)
         │
         ▼
┌──────────────────┐
│ volume_processor │
└────────┬─────────┘
         │
         └─► /volume_stream (Float32: 0-100)
         │
         ▼
┌──────────────────┐
│  Your Node       │
│  (sound detector)│
└──────────────────┘
```

## Performance

- **Latency**: ~50ms (at 20 Hz publish rate)
- **CPU**: Very low (just RMS calculation)
- **Bandwidth**: ~40 bytes/sec (Float32 messages)

Much more efficient than subscribing to raw audio if you only need volume!

## Tips

1. **Calibrate threshold**: Run volume_processor and watch the values, then set your threshold accordingly
2. **Adjust smoothing**: If volume jumps too much, lower smoothing_factor
3. **Monitor in real-time**: Use `ros2 topic echo /volume_stream` to see live values
4. **Combine with other sensors**: Use volume + camera/proximity for better detection

## Troubleshooting

### Volume always 0

- Check if audio_publisher is running
- Verify microphone is working: `arecord -l`
- Check topic: `ros2 topic list` (should show `/audio_stream`)

### Volume too sensitive

Lower the smoothing factor:
```bash
ros2 run audio volume_processor --ros-args -p smoothing_factor:=0.1
```

### Volume not sensitive enough

Increase the smoothing factor:
```bash
ros2 run audio volume_processor --ros-args -p smoothing_factor:=0.7
```

## Next Steps

1. Build and test the volume processor
2. Calibrate your volume threshold
3. Create a sound detector node
4. Integrate with robot actions (movement, LEDs, etc.)

