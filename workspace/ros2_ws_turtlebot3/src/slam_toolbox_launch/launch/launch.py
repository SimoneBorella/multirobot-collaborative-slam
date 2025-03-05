import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    
    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    # Launch descriptions

    slam_toolbox_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': PathJoinSubstitution([get_package_share_directory('slam_toolbox_launch'), 'config', 'mapper_params_online_async.yaml']),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )


    return LaunchDescription([
        # Launch arguments
        log_level_launch_arg,

        # Launch descriptions
        slam_toolbox_launch_description
    ])
