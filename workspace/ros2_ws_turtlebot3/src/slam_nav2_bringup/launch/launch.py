import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Launch descriptions

    slam_toolbox_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': PathJoinSubstitution([get_package_share_directory('slam_nav2_bringup'), 'config', 'mapper_params_online_async.yaml']),
        }.items(),
    )

    nav2_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': os.path.join(get_package_share_directory('nav2_bringup'), 'map', 'map.yaml'),
            'params_file': os.path.join(get_package_share_directory('slam_nav2_bringup'), 'param', 'turtlebot3_waffle.yaml'),
        }.items(),
    )

    # rviz_config_dir = os.path.join(
    #     get_package_share_directory('nav2_bringup'),
    #     'rviz',
    #     'nav2_default_view.rviz')

    # Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_dir],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )


    return LaunchDescription([
        # Launch descriptions
        slam_toolbox_launch_description,
        # nav2_launch_description,
    ])
