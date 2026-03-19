#!/usr/bin/env bash

IMAGE_NAME=g1_slam:humble
ROS_DOMAIN_ID=0

docker rm -f g1_slam 2>/dev/null || true

docker run -d \
  --name g1_slam \
  --restart unless-stopped \
  --net=host \
  --ipc=host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  ${IMAGE_NAME} \
  /ros_entrypoint.sh ros2 launch g1_slam slam.launch.py