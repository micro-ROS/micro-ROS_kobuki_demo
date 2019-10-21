FROM ros2dashingcustom

RUN apt update && apt install curl wget

WORKDIR /
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS.git
WORKDIR /Micro-XRCE-DDS
RUN mkdir build && cd build && cmake .. && make && make install

RUN curl -sSL http://get.gazebosim.org | sh

RUN mkdir -p /crazyflie_demo/src
WORKDIR /crazyflie_demo

RUN cd src && git clone --single-branch --branch crazyflie_demo https://github.com/micro-ROS/micro-ROS_kobuki_demo && mv micro-ROS_kobuki_demo/* . && rm -rf micro-ROS_kobuki_demo

RUN wget https://bitbucket.org/api/2.0/snippets/chapulina/geRKyA/f02dcd15c2c3b83b2d6aac00afe281162800da74/files/ros2.yaml
RUN vcs import src < ros2.yaml
RUN . /ros2_ws/install/setup.sh && rosdep update && rosdep install --from-paths src --ignore-src -r -y && rm ros2.yaml

RUN . /ros2_ws/install/setup.sh && colcon build --symlink-install

RUN cd src/kobuki_twist_keyboard_controller && gcc main.c Twist.c Vector3.c -lmicroxrcedds_client -lmicrocdr -o kobuki_twist_keyboard_controller

RUN cd src/crazyflie_attitude_keyboard_controller && gcc main.c Vector3.c  -lmicroxrcedds_client -lmicrocdr -o crazyflie_attitude_keyboard_controller