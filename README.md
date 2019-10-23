# The micro-ROS crazyflie demo repository


## Dependecies

```
sudo apt update && sudo apt install curl wget
```

## Installation

1. Make sure you have an ROS2 Dashing installation.
2. Create a workspace folder for the demo:

```
mkdir -p crazyflie_demo/src
cd crazyflie_demo
```

3. Clone this repo:
```
git clone --single-branch --branch crazyflie_demo https://github.com/micro-ROS/micro-ROS_kobuki_demo src
```

4. [Install Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install#InstallGazebousingUbuntupackages). Recommended procedure:
```
curl -sSL http://get.gazebosim.org | sh
```

5. [Install gazebo_ros_pkgs (ROS 2)](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros). Recommended procedure:
```
source /opt/ros/dashing/setup.bash
wget https://bitbucket.org/api/2.0/snippets/chapulina/geRKyA/f02dcd15c2c3b83b2d6aac00afe281162800da74/files/ros2.yaml
vcs import src < ros2.yaml
rosdep update && rosdep install --from-paths src --ignore-src -r -y
rm ros2.yaml
```

6. Compile the project:
```
source /opt/ros/dashing/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

7. [Install MicroXCRE-DDS](https://micro-xrce-dds.readthedocs.io/en/latest/installation.html). Recommended procedure:

```
git clone https://github.com/eProsima/Micro-XRCE-DDS.git
cd Micro-XRCE-DDS
mkdir build && cd build
cmake ..
make
sudo make install
```

8. *OPTIONAL* - Compile MicroXRCE Kobuki Twist keyboard controller:

```
cd kobuki_twist_keyboard_controller
gcc main.c Twist.c Vector3.c -lmicrocdr -lmicroxrcedds_client -o kobuki_twist_keyboard_controller
```

8. *OPTIONAL* - Compile MicroXRCE Crazyflie Attitude keyboard simulator:

```
cd crazyflie_attitude_keyboard_controller
gcc main.c Vector3.c -lmicrocdr -lmicroxrcedds_client -o crazyflie_attitude_keyboard_controller
```

## Demos

### Running Kobuki TurtleBot 2 RVIZ Visualizer
```
cd crazyflie_demo
source /opt/ros/dashing/setup.bash
source install/local_setup.bash
ros2 launch micro-ros_kobuki_demo_remote remote_without_control.launch.py
```

### Running Crazyflie 2 RVIZ Visualizer
```
cd crazyflie_demo
source /opt/ros/dashing/setup.bash
source install/local_setup.bash
ros2 launch micro-ros_crazyflie_demo_remote remote_without_control.launch.py
```

### Running MicroXRCE Kobuki TurtleBot 2 Gazebo Simulator
```
cd crazyflie_demo
source /opt/ros/dashing/setup.bash
source install/local_setup.bash
gazebo src/gazebo_kobuki_simulator/worlds/gazebo_ros_kobuki.world
```

### Running Crazyflie Client + MicroXRCE Bridge
```
python3 /crazyflie-clients-python/bin/cfclient
```
### Running Crazyflie MicroXRCE Agent
```
MicroXRCEAgent serial --dev $(cat used_serialport.txt)
```

### Running MicroXRCE Kobuki Twist keyboard controller

Terminal 1:
```
MicroXRCEAgent udp --port 8888
```

Terminal 2:
```
cd kobuki_twist_keyboard_controller
./kobuki_twist_keyboard_controller 127.0.0.1 8888
```

### Running MicroXRCE Crazyflie Attitude keyboard simulator

Terminal 1:
```
MicroXRCEAgent udp --port 8888
```

Terminal 2:
```
cd crazyflie_attitude_keyboard_controller
./crazyflie_attitude_keyboard_controller 127.0.0.1 8888
```



## Docker

Make sure you have the X Server enabled for any connection:
```
xhost +
```

Build the image:
```
cd docker
docker build -t democrazyflie .
```

Run the image:
```
sudo docker run --cap-add=SYS_PTRACE --security-opt seccomp=unconfined --net=host --privileged -it  -e DISPLAY=unix$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev  --name=democrazyflie democrazyflie
```

## Run Demo

Turn off the drone

Execute:

```
sudo docker run --cap-add=SYS_PTRACE --security-opt seccomp=unconfined --net=host --privileged -it  -e DISPLAY=unix$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev  --name=democrazyflie democrazyflie

sudo docker exec democrazyflie /bin/bash /crazyflie_demo/src/scripts/attitude_to_vel.bash
sudo docker exec democrazyflie /bin/bash /crazyflie_demo/src/scripts/crazyflierviz_position.bash
sudo docker exec democrazyflie /bin/bash /crazyflie_demo/src/scripts/kobukigazebo.bash

sudo docker exec democrazyflie /bin/bash /crazyflie_demo/src/scripts/crazyflie_clientbridge.bash
```

Turn on the drone and scan radio device, select the correct address and click connect. MicroXRCE Client to Agent communication may take some seconds.
