# Audio Package - Quick Start

## What This Does

Captures audio from a USB microphone and publishes:
- **Raw audio** to `/audio_stream` for detailed analysis
- **Volume level** to `/volume_stream` (0-100 scale) for sound detection

## Build

```bash
cd /home/vector/Desktop/NUP/MISTI/MISTISoundDetectorProject
colcon build --packages-select audio
source install/setup.bash
```

## Run

### Step 1: Find your microphone

```bash
arecord -l
```

Look for your USB mic (e.g., card 2, device 0 = `hw:2,0`)

### Step 2: Start the audio pipeline

**Recommended: Use the launch file (starts everything)**
```bash
ros2 launch audio audio_with_volume.launch.xml device:=hw:2,0
```

**Or manually:**
```bash
# Terminal 2: Publisher
ros2 run audio audio_publisher --ros-args -p device:="hw:2,0"

# Terminal 3: Volume processor
ros2 run audio volume_processor
```

### Step 3: See the volume!

```bash
# Terminal 4
source install/setup.bash
ros2 topic echo /volume_stream
```

You should see volume values (0-100) updating in real-time!

## What Gets Published

### Raw Audio
- **Topic**: `/audio_stream`
- **Type**: `std_msgs/String` (base64-encoded PCM audio)
- **Format**: 16-bit signed, 44100 Hz, mono
- **Rate**: 20 Hz (20 times/second)

### Volume Level
- **Topic**: `/volume_stream`
- **Type**: `std_msgs/Float32`
- **Range**: 0-100 (0=silence, 100=very loud)
- **Rate**: 20 Hz (same as audio)

## Use It in Your Code

### Option 1: Use Volume (Simple - Recommended!)

```python
from std_msgs.msg import Float32

def volume_callback(msg):
    volume = msg.data  # 0-100
    
    if volume > 40.0:
        print(f"Loud sound! Volume: {volume:.1f}")
        # Do something: move robot, flash LEDs, etc.

# Subscribe
self.sub = self.create_subscription(Float32, 'volume_stream', volume_callback, 10)
```

### Option 2: Use Raw Audio (Advanced)

```python
from std_msgs.msg import String
import base64
import numpy as np

def audio_callback(msg):
    # Decode audio
    audio_data = base64.b64decode(msg.data)
    samples = np.frombuffer(audio_data, dtype=np.int16)
    
    # Your custom analysis (FFT, etc.)
    # ...

# Subscribe
self.sub = self.create_subscription(String, 'audio_stream', audio_callback, 10)
```

## Docker

```bash
# Run complete pipeline (publisher + volume processor)
docker run --rm --privileged -v /dev:/dev --network host \
  your-image \
  ros2 launch audio audio_with_volume.launch.xml device:=hw:2,0

# Or just publisher
docker run --rm --privileged -v /dev:/dev --network host \
  your-image \
  ros2 run audio audio_publisher --ros-args -p device:="hw:2,0"
```

## Files

- Publisher: `audio/audio.py`
- Volume processor: `audio/volume_processor.py` ⭐
- Subscriber example: `audio/audio_subscriber.py`
- Launch files: `launch/`
- Full docs: `README.md`, `VOLUME_PROCESSOR.md`

