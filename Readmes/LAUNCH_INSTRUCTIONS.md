# Sound Hunter Robot - Launch Instructions

## Docker Build & Run

### 1. Build the Docker Image

```bash
cd /home/vector/Desktop/NUP/MISTI/MISTISoundDetectorProject
docker build -t sound-hunter .
```

### 2. Run the Container

```bash
docker run -it --rm \
  --name sound-hunter-robot \
  --privileged \
  --device=/dev/snd \
  -v /home/vector/Desktop/NUP/MISTI/MISTISoundDetectorProject:/ws \
  -e VEHICLE_NAME=duckie05 \
  -e LEFT_VEL_MULT=1.0 \
  -e RIGHT_VEL_MULT=1.0 \
  sound-hunter bash
```

 docker run  -d --network=host -v /dev/shm:/dev/shm -v ~/MISTISoundDetectorProject:/ws --privileged --name vec  vec_image:latest

docker run -d --network=host -v /dev/shm:/dev/shm  --privileged \
  --device=/dev/snd \
  -v .:/ws \
  -e VEHICLE_NAME=duckie05 \
  -e LEFT_VEL_MULT=1.0 \
  -e RIGHT_VEL_MULT=1.0 \
  sound-hunter /bin/bash/


**Notes:**
- `--privileged` gives access to audio devices
- `--device=/dev/snd` mounts audio devices
- `-v` mounts your workspace for live code editing
- `-e VEHICLE_NAME=duckie05` sets robot name

### 3. Inside Container: Launch Tmux

```bash
# If you need to rebuild (after code changes):
cd /ws
rm -rf build log install
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Launch the tmux session
./launch_robot.sh
```

## Tmux Layout

### Window 1: Control (default view)
```
┌─────────────────┬─────────────────┐
│  Movement       │  Volume Echo    │
│  (movement.py)  │  (monitor)      │
├─────────────────┼─────────────────┤
│  Blinker        │  Joystick       │
│  (blinker.py)   │  (launch.xml)   │
└─────────────────┴─────────────────┘
```

### Window 2: Audio
```
┌─────────────────┬─────────────────┐
│  Audio Capture  │  Volume Proc    │
│  (audio.py)     │  (volume_proc)  │
└─────────────────┴─────────────────┘
```

## Tmux Commands

- **Switch windows**: `Ctrl+b` then `1` or `2`
- **Navigate panes**: `Ctrl+b` then arrow keys
- **Zoom pane**: `Ctrl+b` then `z` (zoom in/out)
- **Scroll mode**: `Ctrl+b` then `[` (arrow keys to scroll, `q` to exit)
- **Kill pane**: `Ctrl+b` then `x`
- **Detach**: `Ctrl+b` then `d`
- **Reattach**: `tmux attach -t robot`
- **Kill session**: `tmux kill-session -t robot`

## Starting the Robot

In each pane, the command is ready - just **press Enter** to start:

### Recommended startup order:

1. **Window 2 (audio)** - Start both:
   - Left: Audio capture (press Enter)
   - Right: Volume processor (press Enter)
   - Wait for "Volume processor started" message

2. **Window 1 (control)** - Start nodes:
   - Left top: Movement (press Enter)
   - Left bottom: Blinker (press Enter)
   - Right top: Volume echo (press Enter) - to monitor sound levels
   - Right bottom: Joystick (optional, for manual control)

## Testing

### Check if audio is working:
```bash
# In any terminal
ros2 topic echo /duckie05/volume_stream
```

You should see numbers like:
```
data: 35.2
---
data: 42.8
---
```

### Make a sound
- Clap or yell near the microphone
- Volume should jump to 60-100%
- Robot should:
  1. Wait 2 seconds (RED LED)
  2. Rotate 360° scanning (YELLOW LED)
  3. Rotate to best direction (ORANGE LED)
  4. Move forward (GREEN LED)
  5. Rescan every 10s or if sound decreases

## LED Colors

| Color | State | Meaning |
|-------|-------|---------|
| 🔵 BLUE | Idle | Waiting for sound |
| 🔴 RED | Sound detected | Waiting before scan |
| 🟡 YELLOW | Scanning | Finding loudest direction |
| 🟠 ORANGE | Rotating | Aligning to target |
| 🟢 GREEN | Moving | Approaching sound source |

## Tuning Parameters

### Movement parameters:
```bash
ros2 run movement movement.py --ros-args \
  -p volume_threshold:=30.0 \
  -p scan_speed:=0.5 \
  -p forward_speed:=0.6
```

### Audio parameters:
```bash
ros2 run audio audio.py --ros-args \
  -p publish_rate:=20.0 \
  -p device:="plughw:2,0"
```

### Volume processor parameters:
```bash
ros2 run audio volume_processor.py --ros-args \
  -p smoothing_factor:=1.0 \
  -p volume_scale_max:=5000.0
```

## Troubleshooting

### Audio device not found
```bash
# List audio devices
arecord -l

# Test device
arecord -D plughw:2,0 -d 2 test.wav
```

### Robot not responding to sound
- Check volume level: `ros2 topic echo /duckie05/volume_stream`
- Lower threshold: `-p volume_threshold:=15.0`
- Check microphone is not muted

### Tmux script fails
```bash
# Make sure it's executable
chmod +x /ws/launch_robot.sh

# Run manually:
cd /ws
./launch_robot.sh
```

### Rebuild after code changes
```bash
cd /ws
rm -rf build log install
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

## Quick Reference

### Behavior Flow
```
Sound > threshold
    ↓
Wait 2 seconds (RED)
    ↓
360° scan (YELLOW) - find loudest direction
    ↓
Rotate to target (ORANGE)
    ↓
Move forward (GREEN) - 10 seconds
    ↓
If range > 15cm and 10s elapsed:
    → Rescan ±45° (YELLOW)
    → Repeat
    ↓
If sound decreasing:
    → Rescan ±45° (YELLOW)
    → Repeat
    ↓
If range ≤ 15cm:
    → STOP (BLUE)
```

## File Structure
```
/ws/
├── launch_robot.sh          # Tmux launch script
├── src/
│   ├── audio/               # Audio capture & processing
│   ├── movement/            # Robot movement logic
│   └── blinker/             # LED control
├── requirements-apt.txt     # System packages
├── requirements-python.txt  # Python packages
└── Dockerfile               # Container definition
```

