# Frequency-Specific Volume Detection

Detect volume of specific frequency ranges (e.g., 500Hz tone from phone)

## How It Works

Uses **FFT (Fast Fourier Transform)** to analyze audio frequency spectrum and measure volume of a specific frequency range.

```
Audio samples → FFT → Frequency spectrum → Filter target freq → Calculate RMS → Volume
```

## Quick Start

### 1. Run Audio Capture
```bash
ros2 run audio audio.py
```

### 2. Run Frequency Volume Processor
```bash
# Default: 500Hz ±50Hz (450-550Hz range)
ros2 run audio frequency_volume_processor
```

### 3. Play 500Hz Tone
- Use tone generator app on phone
- Or online: https://onlinetonegenerator.com/
- Set to 500Hz, play near robot

### 4. Monitor Output
```bash
# Watch frequency-specific volume
ros2 topic echo /duckie05/frequency_volume_stream

# Watch total volume (all frequencies)
ros2 topic echo /duckie05/total_volume_stream
```

## Parameters

### Target Frequency
```bash
# Detect 1000Hz tone
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=1000.0

# Detect 250Hz tone
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=250.0
```

### Frequency Bandwidth
```bash
# Narrow bandwidth (±25Hz = 475-525Hz for 500Hz target)
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=500.0 \
  -p frequency_bandwidth:=50.0

# Wide bandwidth (±100Hz = 400-600Hz for 500Hz target)
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=500.0 \
  -p frequency_bandwidth:=200.0
```

### Other Parameters
```bash
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=500.0 \
  -p frequency_bandwidth:=50.0 \
  -p smoothing_factor:=1.0 \          # 1.0 = instant, 0.9 = smooth
  -p normalize_to_100:=true \         # Scale to 0-100%
  -p volume_scale_max:=5000.0         # Max RMS for 100%
```

## Output Topics

### `/duckie05/frequency_volume_stream`
Volume of target frequency only (Float32)

**Example:**
```
data: 85.3   # 500Hz tone is loud (85.3%)
---
data: 12.5   # Background noise at 500Hz (12.5%)
---
data: 95.0   # Very loud 500Hz tone (95%)
```

### `/duckie05/total_volume_stream`
Total volume of all frequencies (Float32)

**Example:**
```
data: 45.2   # Overall sound level
---
data: 60.8   # Louder overall
```

## Use Cases

### 1. Phone Tone Detection (500Hz)
```bash
# Robot follows 500Hz tone from your phone
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=500.0 \
  -p frequency_bandwidth:=50.0

# Then in movement.py, subscribe to:
# /duckie05/frequency_volume_stream instead of /duckie05/volume_stream
```

### 2. Whistle Detection (2000-4000Hz)
```bash
# Detect human whistle (high frequency)
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=3000.0 \
  -p frequency_bandwidth:=2000.0
```

### 3. Low Frequency Rumble (50-200Hz)
```bash
# Detect bass/rumble sounds
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=125.0 \
  -p frequency_bandwidth:=150.0
```

### 4. Voice Detection (300-3400Hz)
```bash
# Detect human voice frequencies
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=1850.0 \
  -p frequency_bandwidth:=3100.0
```

## Testing

### Generate Test Tones

**Online generators:**
- https://onlinetonegenerator.com/
- https://www.szynalski.com/tone-generator/

**Phone apps:**
- "Tone Generator" (Android)
- "Function Generator" (iOS)

### Test Procedure

```bash
# 1. Start audio capture
ros2 run audio audio.py

# 2. Start frequency detector (500Hz)
ros2 run audio frequency_volume_processor --ros-args -p target_frequency:=500.0

# 3. In another terminal, watch output
ros2 topic echo /duckie05/frequency_volume_stream

# 4. Play different frequencies and observe:
# - 500Hz tone → HIGH volume (80-100%)
# - 1000Hz tone → LOW volume (0-10%)
# - Background noise → LOW volume (5-20%)
# - 500Hz + noise → MEDIUM-HIGH volume (60-90%)
```

## Log Output Example

```
[INFO] [frequency_volume_processor]: Frequency Volume Processor started
[INFO] [frequency_volume_processor]:   Target frequency: 500.0 Hz
[INFO] [frequency_volume_processor]:   Frequency range: 450.0 - 550.0 Hz
[INFO] [frequency_volume_processor]:   Smoothing factor: 1.0
[INFO] [frequency_volume_processor]:   Normalize to 0-100: True
[INFO] [frequency_volume_processor]: Waiting for audio stream...
[INFO] [frequency_volume_processor]: Received metadata: 16000Hz, 1ch
[INFO] [frequency_volume_processor]: 500Hz: 12.3% | Total: 35.8% | Ratio: 34% (packet #10)
[INFO] [frequency_volume_processor]: 500Hz: 85.6% | Total: 78.2% | Ratio: 109% (packet #20)
[INFO] [frequency_volume_processor]: 500Hz: 92.1% | Total: 88.5% | Ratio: 104% (packet #30)
```

**Understanding the ratio:**
- `Ratio: 34%` = Target frequency is 34% of total volume (low, mostly other sounds)
- `Ratio: 104%` = Target frequency dominates (your tone is louder than everything else!)

## Integration with Movement

### Option 1: Use frequency volume directly
Modify `movement.py`:

```python
# Change subscription from volume_stream to frequency_volume_stream
self.create_subscription(
    Float32,
    f'/{self.vehicle_name}/frequency_volume_stream',  # Changed!
    self.sound_callback,
    1
)
```

Now robot only responds to 500Hz tone from your phone, ignoring other sounds!

### Option 2: Use ratio (advanced)
Robot responds only when specific frequency dominates:

```python
# Subscribe to both topics
self.freq_volume = 0.0
self.total_volume = 0.0

def freq_callback(self, msg):
    self.freq_volume = msg.data

def total_callback(self, msg):
    self.total_volume = msg.data
    
def control_loop(self):
    # Only respond if 500Hz is >70% of total sound
    ratio = (self.freq_volume / self.total_volume) if self.total_volume > 10 else 0
    if ratio > 0.7:
        # 500Hz tone detected!
        self.start_scan()
```

## Frequency Reference

| Frequency | Sound Example |
|-----------|---------------|
| 50-60 Hz | Electrical hum, deep bass |
| 100-200 Hz | Thunder, large engine |
| 250-500 Hz | Male voice fundamental |
| 500-1000 Hz | Phone tones, female voice |
| 1000-2000 Hz | Whistle (low) |
| 2000-4000 Hz | Whistle (high), bird chirp |
| 4000-8000 Hz | Sibilants (s, sh sounds) |
| 8000+ Hz | Very high pitched sounds |

## Troubleshooting

### "Low volume even with loud 500Hz tone"
- Check `volume_scale_max`: Try `-p volume_scale_max:=2000.0` (lower = more sensitive)
- Check frequency: Make sure phone is playing exactly 500Hz
- Check bandwidth: Try wider `-p frequency_bandwidth:=100.0`

### "High volume on background noise"
- Narrow bandwidth: `-p frequency_bandwidth:=30.0`
- Check frequency: Background noise might have energy at 500Hz

### "Inconsistent readings"
- Add smoothing: `-p smoothing_factor:=0.8`
- Increase sample rate in audio.py: `-p publish_rate:=30.0`

### "Can't detect any frequency"
```bash
# Check audio is working
ros2 topic echo /duckie05/volume_stream

# Check metadata received
ros2 topic echo /duckie05/audio_metadata

# Check FFT is running
ros2 topic echo /duckie05/frequency_volume_stream
```

## Complete Example: 500Hz Phone Follower

```bash
# Terminal 1: Audio capture
ros2 run audio audio.py

# Terminal 2: Frequency detector (500Hz)
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=500.0 \
  -p frequency_bandwidth:=50.0

# Terminal 3: Movement (modify to use frequency_volume_stream)
ros2 run movement movement.py

# Terminal 4: Monitor
ros2 topic echo /duckie05/frequency_volume_stream

# Now: Play 500Hz tone on phone near robot
# Robot will detect and follow the tone!
```

## Advanced: Multiple Frequencies

Run multiple processors for different frequencies:

```bash
# Detect 500Hz (phone tone)
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=500.0 \
  --ros-remap __node:=freq_500

# Detect 1000Hz (different tone)
ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=1000.0 \
  --ros-remap __node:=freq_1000

# Now you have two topics:
# /duckie05/frequency_volume_stream (from freq_500)
# /duckie05/frequency_volume_stream (from freq_1000)
# Note: You'd need to rename topics to avoid collision
```

