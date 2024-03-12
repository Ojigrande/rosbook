FROM ros:humble

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV TZ=Europe/Moscow

ENV ROS_DISTRO humble
ARG USER_ID
ARG GROUP_ID

RUN apt-get update && apt-get install -y --no-install-recommends \
  curl \
  git

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-demo-nodes-cpp \
    ros-humble-rqt-graph \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    ros-humble-rqt-tf-tree

RUN mkdir -p /opt/rosbook/src && \
    cd /opt/rosbook/src && \
    git clone -b humble-devel https://github.com/fmrico/book_ros2.git && \
    vcs import . < book_ros2/third_parties.repos && \
    cd /opt/rosbook && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN cd /opt/rosbook && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# init Colcon workspace
#RUN apt-get update && apt-get install -y --no-install-recommends \
#  python3-pip \
#  python3-apt \
#  python3-colcon-common-extensions \
#  curl \
#  libpcl-dev \
#  git
#  && pip3 install -U transforms3d

# install additional ros packages
#RUN apt-get install -y --no-install-recommends \
#  ros-${ROS_DISTRO}-fastrtps \
#  ros-${ROS_DISTRO}-fastrtps-cmake-module \
#  ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
#  ros-${ROS_DISTRO}-rmw-fastrtps-shared-cpp \
#  ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-c \
#  ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-cpp \
#  ros-${ROS_DISTRO}-velodyne-msgs \
#  ros-${ROS_DISTRO}-nav2-msgs \
#  ros-${ROS_DISTRO}-map-msgs \
#  ros-${ROS_DISTRO}-realsense2-camera-msgs \
#  && rm -rf /var/lib/apt/lists/*


# install additional packages
#RUN apt-get update && apt-get install -y --no-install-recommends \
#  bmon \
#  && rm -rf /var/lib/apt/lists/*
#
#RUN pip install fastapi uvicorn

#
#COPY ./commander/commander /ros2_ws/src/commander
#COPY ./navedu_msgs /ros2_ws/src/navedu_msgs
#COPY ./commander/external_msgs /ros2_ws/src/external_msgs 
## RUN apt update && apt install -y ros-humble-rmw-cyclonedds-cpp
## ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /ros2_ws/ && colcon build --symlink-install'


#COPY ./docker/commander/ros_entrypoint.sh /
#RUN pip install scipy
#WORKDIR /opt
#RUN git clone https://github.com/rkorv/tgcli
#RUN cd tgcli/tgcli && pip install --no-cache-dir .
