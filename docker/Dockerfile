FROM ros:dashing

RUN apt update && apt install -y software-properties-common
RUN add-apt-repository ppa:team-gcc-arm-embedded/ppa
RUN apt update && apt install -y curl wget ros-dashing-desktop gcc-arm-embedded dfu-util

WORKDIR /
RUN git clone -b dashing https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /Micro-XRCE-DDS-Agent
RUN . /opt/ros/dashing/setup.sh && mkdir build && cd build && cmake .. && make && make install && ldconfig /usr/local/lib/

RUN curl -sSL http://get.gazebosim.org | sh

RUN apt update && apt install -y ros-dashing-gazebo-ros-pkgs

RUN mkdir -p /crazyflie_demo/src
WORKDIR /crazyflie_demo

RUN git clone --single-branch --branch crazyflie_demo https://github.com/micro-ROS/micro-ROS_kobuki_demo src 

RUN . /opt/ros/dashing/setup.sh && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/dashing/setup.sh && colcon build --symlink-install

RUN cd src/kobuki_twist_keyboard_controller && gcc main.c Twist.c Vector3.c -lmicroxrcedds_client -lmicrocdr -o kobuki_twist_keyboard_controller

RUN cd src/crazyflie_attitude_keyboard_controller && gcc main.c Point32.c  -lmicroxrcedds_client -lmicrocdr -o crazyflie_attitude_keyboard_controller

WORKDIR /
RUN git clone --single-branch --branch Micro-XRCE-DDS_Bridge https://github.com/eProsima/crazyflie-clients-python.git
RUN apt update &&  apt install -y python3 python3-pip python3-pyqt5 python3-pyqt5.qtsvg
RUN cd /crazyflie-clients-python && pip3 install -e .

RUN git clone https://github.com/bitcraze/crazyflie-lib-python.git
RUN cd crazyflie-lib-python && ./setup_linux.sh