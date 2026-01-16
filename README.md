# MISTI Sound Detector Robot

Reacting to the loudest sound.

## Initial roles  
Valentin: audio : audio_stream <- stream (Valentin's work to provide the audio ) 
Eugen: procees audio_stream and create a new publosher with only VOLUME of sound. Publisher name : volume_stream.
Christos: Create a class, that gets a volume from volume_stream, write some logic in function `movement`. Let it just rotate and make some moves. Publish to  self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)

## BUILD and RUN instructions 

`docker build -t vec_image .  `
OLD:`docker run  -d --network=host -v /dev/shm:/dev/shm -v ~/MISTISoundDetectorProject:/ws --privileged --name vec  vec_image:latest`
NEW: 
```bash
cd MISTI
docker run -d --network=host --privileged -v /dev/shm:/dev/shm --device=/dev/snd -v $(pwd):/ws -e VEHICLE_NAME=duckie05 -e USER_NAME=duckie05 --name qwe vec_image bash -c "while true; do sleep 3600; done"

```
`docker exec -it vec /bin/bash`
REMOVE(if needed):`docker rm -f vec_container`

## duckie container
rebuild if code changed
```bash
cd /ws
rm -rf build log install 
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
launch tmux with commands (2 windows, 6 terminals)
```bash
./launch_robot.sh
```

#### ROS2 RUN commands
```bash

to see logs:
2>&1 | tee movement_logs.txt


ros2 run audio audio.py --ros-args \
  -p publish_rate:=10.0 \
  -p period_size:=1600
  -p device:="plughw:2,0"

# not used now, outdated
ros2 run audio volume_processor.py --ros-args \
  -p smoothing_factor:=1.0 \
  -p volume_scale_max:=5000.0

ros2 run audio frequency_volume_processor.py --ros-args -p target_frequency:=2000.0 -p frequency_bandwidth:=10.0 -p log_interval:=1 -p  freq_volume_scale_max:=332167 -p volume_scale_max:=20000.0

ros2 run movement movement.py --ros-args -p left_vel_mult:=0.7 -p right_vel_mult:=0.5 -p volume_threshold:=10.0 -p scan_360_duration:=4.0 -p scan_90_duration:=1.1 -p scan_speed:=0.9 -p rotate_speed:=0.7 -p forward_speed:=1.2 -p sound_decrease_threshold:=10.0 -p target_range:=0.3


ros2 launch master_launch launch.xml

ros2 run blinker blinker.py
```

### TroubleShoting
#### SOLVE to alsaaudio (in duckie container):
```bash
# In your container
apt-get update
apt-get install -y libasound2-dev
pip3 install --upgrade pyalsaaudio

cd /ws
colcon build --packages-select audio
source install/setup.bash
```
- different lib:
```bash
pip3 install sounddevice
RUN(plughw instead)
ros2 run audio audio.py --ros-args -p device:="plughw:2,0" -p rate:=16000
```

#### Frequencies and sensetivities

```bash

# If frequency is always >80%, increase scale:
ros2 run audio frequency_volume_processor --ros-args \
  -p freq_volume_scale_max:=8000.0


```

