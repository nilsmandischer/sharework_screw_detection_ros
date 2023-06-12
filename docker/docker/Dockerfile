FROM ros:noetic-perception

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
      wget unzip \
      git \
      bash-completion \
      build-essential \
      sudo \
      python3-catkin-tools \
      build-essential g++ \
      ros-noetic-rviz ros-noetic-rqt ros-noetic-rqt-common-plugins \
      gedit

RUN mkdir /home/Downloads 
RUN mkdir /catkin_ws
RUN mkdir /catkin_ws/src
RUN mkdir /images

#ScrewDetection
WORKDIR /catkin_ws/src
RUN wget -O sharework_screw_detection_ros.zip \
https://github.com/nilsmandischer/sharework_screw_detection_ros/archive/refs/heads/master.zip && \
      unzip sharework_screw_detection_ros.zip -d sharework_screw_detection_ros

RUN mkdir /catkin_ws/src/sharework_screw_detection_ros/modules
WORKDIR /catkin_ws/src/sharework_screw_detection_ros/modules
RUN wget -O sharework_screw_detection.zip \
https://github.com/nilsmandischer/sharework_screw_detection/archive/refs/heads/master.zip && \
      unzip sharework_screw_detection.zip -d sharework_screw_detection

WORKDIR /catkin_ws/src/sharework_screw_detection_ros/modules/sharework_screw_detection/screw_detection
RUN mkdir build && cd build && cmake .. && make

#Build the workspace
WORKDIR /catkin_ws
RUN catkin build

#Remove unneeded files
RUN rm -r /home/Downloads

#Set up ROS
WORKDIR /catkin_ws
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/.bashrc
RUN echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc
