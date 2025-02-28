#!/bin/bash
source install/setup.bash

x_init=-2.0
y_init=0.0
z_init=0.0
R_init=0.0
P_init=0.0
Y_init=0.0

ros_gz_rviz=False
nav2_bringup_rviz=True


ros2 launch launch launch.py \
    x_init:=$x_init \
    y_init:=$y_init \
    z_init:=$z_init \
    R_init:=$R_init \
    P_init:=$P_init \
    Y_init:=$Y_init \
    \
    ros_gz_rviz:=$ros_gz_rviz \
    nav2_bringup_rviz:=$nav2_bringup_rviz
