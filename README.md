# MISTI Sound Detector Robot

This project was developed as a part of 5-days *MIT's MISTI Global Teaching Labs* program. Jan 2026.
- [Presentation slides](https://docs.google.com/presentation/d/1izpK_8jd24ZRqsdoBZ5pSNQUvmmD8BSseZpE0pxE2c8/edit?usp=sharing) 

- Robot’s task is to detect the sound, scan for the direction it is coming from and move towards it until it reaches the target. Moving forward is not accurate, so every 10 seconds or when sound volume decreased by some value, robot does a mini-rescan to increase detection precision.
- The robot is listening to 2000Hz sound by default in order to cancel out all noise. It also stops 30sm before target and changes LED color lights depending on its current state(waiting,scanning,moving forward and etc.). 
- In future we plan to add AI to detected the *HELP* message in different languages, *PID* regulator and *two microphones* in order to detect location quickly(without scanning), dynamicly(sound object may move) and  accurately(moving forward and rotating is executed as expected in real world with a help of *PID* regulator). 

Its purpose in Cyprus is to help locate missing people in areas like the Troodos Mountains or forests, and to find anyone in danger during wildfires.


## BUILD and RUN instructions 



```bash
cd MISTISoundDetectorProject

docker build -t vec_image .  

docker run -d --network=host --privileged -v /dev/shm:/dev/shm --device=/dev/snd -v $(pwd):/ws -e VEHICLE_NAME=duckie05 -e USER_NAME=duckie05 --name qwe vec_image bash -c "while true; do sleep 3600; done"

docker exec -it vec /bin/bash

```

REMOVE(if needed):`docker rm -f vec_container`

## Inside duckie container

launch tmux with commands (2 windows, 6 terminals)
```bash
./launch_robot.sh
```


rebuild if code changed
```bash
cd /ws
rm -rf build log install 
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash
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

ros2 run movement movement.py --ros-args -p left_vel_mult:=0.7 -p right_vel_mult:=0.5 -p volume_threshold:=10.0 -p scan_360_duration:=3.0 -p scan_90_duration:=1.7 -p scan_speed:=0.9 -p rotate_speed:=0.7 -p forward_speed:=1.2 -p sound_decrease_threshold:=10.0 -p target_range:=0.3


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

