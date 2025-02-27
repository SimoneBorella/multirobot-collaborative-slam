import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Launch arguments

    # sim_type_launch_arg = DeclareLaunchArgument(
    #     'sim_type',
    #     default_value='turtlebot3_waffle',
    #     choices=['diff_drive', 'turtlebot3_waffle'],
    #     description='Select simulation type to launch'
    # )



    # Launch descriptions

    ros_gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_bringup'), 'launch','launch.py')
        ]),
        launch_arguments={
            # 'sim_type': LaunchConfiguration('sim_type'),
        }.items()
    )

    nav2_bringup_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_nav2_bringup'), 'launch', 'launch.py')
        ]),
        launch_arguments={
        }.items(),
    )
    

    return LaunchDescription([
        # Launch arguments
        # sim_type_launch_arg,

        # Launch descriptions
        ros_gz_launch_description,
        nav2_bringup_launch_description
    ])

