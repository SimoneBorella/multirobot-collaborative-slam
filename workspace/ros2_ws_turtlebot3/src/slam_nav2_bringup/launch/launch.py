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

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
    )

    # Launch descriptions

    slam_toolbox_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': PathJoinSubstitution([get_package_share_directory('slam_nav2_bringup'), 'config', 'mapper_params_online_async.yaml']),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    nav2_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': os.path.join(get_package_share_directory('slam_nav2_bringup'), 'maps', 'map.yaml'),
            'params_file': os.path.join(get_package_share_directory('slam_nav2_bringup'), 'param', 'turtlebot3_waffle.yaml'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', os.path.join(get_package_share_directory('slam_nav2_bringup'), 'config', 'nav2.rviz'),
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    LaunchConfiguration('rviz'), " == True",
                ]
            )
        )
    )


    return LaunchDescription([
        # Launch arguments
        rviz_launch_arg,
        log_level_launch_arg,

        # Launch descriptions
        slam_toolbox_launch_description,
        nav2_launch_description,
        rviz_node
    ])
