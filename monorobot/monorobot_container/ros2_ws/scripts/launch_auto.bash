#!/bin/bash
clear

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=2
export CAMERA_MODEL=oakd

source /opt/ros/humble/setup.bash
source install/setup.bash

namespace=robot_12
bag_record=True
rviz_nav2=True
log_level=error

exec ros2 launch launch launch.py \
    namespace:=$namespace \
    bag_record:=$bag_record \
    rviz_nav2:=$rviz_nav2 \
    log_level:=$log_level
