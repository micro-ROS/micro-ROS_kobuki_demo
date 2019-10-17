FROM ros:dashing

RUN apt update && apt install curl wget

RUN git clone https://github.com/eProsima/Micro-XRCE-DDS.git
WORKDIR /Micro-XRCE-DDS
RUN mkdir build && cd build && cmake .. && make && make install

RUN mkdir -p crazyflie_demo/src
WORKDIR /crazyflie_demo

RUN git clone --single-branch --branch crazyflie_demo https://github.com/micro-ROS/micro-ROS_kobuki_demo src

RUN curl -sSL http://get.gazebosim.org | sh

RUN wget https://bitbucket.org/api/2.0/snippets/chapulina/geRKyA/f02dcd15c2c3b83b2d6aac00afe281162800da74/files/ros2.yaml
RUN vcs import src < ros2.yaml
RUN source /opt/ros/dashing/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y && rm ros2.yaml

RUN source /opt/ros/dashing/setup.bash && colcon build --symlink-install

WORKDIR /

RUN git clone https://github.com/eProsima/Micro-XRCE-DDS.git
WORKDIR /Micro-XRCE-DDS
RUN mkdir build && cd build && cmake .. && make && make install

RUN cd kobuki_twist_keyboard_controller && gcc main.c Twist.c Vector3.c -lmicrocdr -lmicroxrcedds_client -o kobuki_twist_keyboard_controller

RUN cd crazyflie_attitude_keyboard_controller && gcc main.c Vector3.c -lmicrocdr -lmicroxrcedds_client -o crazyflie_attitude_keyboard_controller