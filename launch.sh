#!/bin/sh

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

if [ ! -f "$XAUTH" ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

#sudo xhost +local:root

#           --shm-size=1gb \
docker run --privileged -it \
           --env="XAUTHORITY=${XAUTH}" \
           --env="DISPLAY=${DISPLAY}" \
           --env=TERM=xterm-256color \
           --env=QT_X11_NO_MITSHM=1 \
           --volume=$XSOCK:$XSOCK:rw \
           --volume=$XAUTH:$XAUTH:rw \
           --volume=$(pwd)/testros:/ros:rw \
           --net=host \
           --runtime=nvidia \
           --gpus all \
           --ipc=host \
           --ulimit memlock=-1 \
           --ulimit stack=67108864 \
           --workdir /ros \
           myros:1 \
           bash

#            osrf/ros:humble-desktop \