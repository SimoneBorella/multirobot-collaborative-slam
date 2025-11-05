#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

namespace=robot_12
rviz_config=/ros2_ws/install/nav2_launch/share/nav2_launch/rviz/nav2_view.rviz
log_level=error

ros2 launch nav2_launch rviz_launch.py \
    namespace:=$namespace \
    rviz_config:=$rviz_config \
    log_level:=$log_level