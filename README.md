### PROJECT Arhitecture


Valentin: audio : audio_stream <- stream (Valentin's work to provide the audio ) 
Eugen: procees audio_stream and create a new publosher with only VOLUME of sound. Publisher name : volume_stream.
Christos: Create a class, that gets a volume from volume_stream, write some logic in function `movement`. Let it just rotate and make some moves. Publish to  self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)

## RUN it 

### BUILD duckie container:
 
`docker run  -d --network=host -v /dev/shm:/dev/shm -v ~/MISTISoundDetectorProject:/ws --privileged --name vec  vec_image:latest`
`docker exec -it vec /bin/bash`

#### duckie container
```bash
cd /ws
rm -rf build log install 
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

#### ROS2 RUN commands
```bash
ros2 run audio audio.py --ros-args -p publish_rate:=5.0

ros2 run audio volume_processor.py

ros2 run movement movement.py

ros2 run blinker blinker.py
```

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

# Skeleton Code for Project

In this template repo we have a blank package and blank node structure with some relatively standard edits. This is a good starting place for the project and you can add all your own functionalities by duplicating blank nodes and packages etc. The structure is similar to the example repo structures:

```
src                   ← ROS workspace
└── blank_package/           ← ROS package
    ├── blank_package/       ← Python module 
    │   └── blank_node.py    ← Your actual ROS node code
    ├── resource/
    │   └── blinker       
    ├── package.xml          ← ROS metadata (we add dependencies)
    ├── setup.py             ← Python packaging + ROS entry points (manually add line for each node)
    └── setup.cfg        
```


### This is a template repo so the steps are a bit different. You can copy and clone it as follows:
- One person per team do this (they will be the owner of the repo):
  - In Github web, click the green ``Use this template" button and then click ``Create a new repository".
  - Have the owner be yourself (you personal github account) and name the repository ``MIT GTL Project"
  - Make visibility public (if not already)
  - Click the green ``Create repository" button.
  - Go to ``Setting" in the navigation bar.
  - Click ``Collaborator" on the left and add you teammates' emails via the ``Add people".
- All other teammates accept the invite. Now everyone has full access to the repo.
- Everyone now clone as before:
  - Open terminal
  - Navigate to where you want to put your cloned repo
  - (hopefully you have an SSH key from in class activity, if not follow instruction on Google)
  - From Github web, click green code button and copy SSH link
  - In terminal, do ``` git clone [INSERT LINK] ```


### Here are relevent git commands interact with the repo and make/receive updates:
- ```git pull``` to get latest changes on main branch 
- To put your own changes into the repo:
    - ```git add .``` to stage all files changed
    - ```git status``` to check which files are going to be sent to the repo
    - ```git commit -m "[INFORMATIVE MESSAGE]"``` to explain what you're going send
    - ```git push``` to finally send everything
- Be careful with merge conflict if multiple people work on the same file and try to push conflicting things. These may have to be manually resolved - it's annoying to handle!



### How to run things:
1. #### Local Server
    1. You'll want to save your changes to git
        1. ```git add .``` to stage all files
        2. ```git commit -m "[INFORMATIVE MESSAGE]"``` to explain what you are sending
        3. ```git push``` to finally send everything
    2. Connect to remote server via ssh
       ```bash
        ssh user@rpi-server.local
       ```
       with the password ```quackquack```
2. #### Remote Server
    1. You'll want to get your changes from git
        1. ```git pull``` to get latest changes on main branch
    2. Run ```make build```
    3. Run ```make run```
3. #### Docker Container Within Remote Server
    1. Launch the necessary ros2 packages using
       ```bash
       ros2 run package node
       ```
       ```bash
       ros2 launch launchPackage launchFile
       ```
   

          
*Do see cheat sheet as well for more commands
