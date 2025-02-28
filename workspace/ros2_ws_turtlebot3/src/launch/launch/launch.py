import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Launch arguments

    x_init_launch_arg = DeclareLaunchArgument(
        'x_init',
        default_value='0.0'
    )
    y_init_launch_arg = DeclareLaunchArgument(
        'y_init',
        default_value='0.0'
    )
    z_init_launch_arg = DeclareLaunchArgument(
        'z_init',
        default_value='0.0'
    )
    R_init_launch_arg = DeclareLaunchArgument(
        'R_init',
        default_value='0.0'
    )
    P_init_launch_arg = DeclareLaunchArgument(
        'P_init',
        default_value='0.0'
    )
    Y_init_launch_arg = DeclareLaunchArgument(
        'Y_init',
        default_value='0.0'
    )

    pose_cov_init_launch_arg = DeclareLaunchArgument(
        'pose_cov_init',
        default_value='[ \
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0, \
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0, \
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0, \
            0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, \
            0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, \
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0001 \
        ]',
    )

    ros_gz_rviz_launch_arg = DeclareLaunchArgument(
        'ros_gz_rviz',
        default_value='True'
    )

    nav2_bringup_rviz_launch_arg = DeclareLaunchArgument(
        'nav2_bringup_rviz',
        default_value='True'
    )







    # Launch descriptions

    ros_gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_bringup'), 'launch','launch.py')
        ]),
        launch_arguments={
            'x_init': LaunchConfiguration('x_init'),
            'y_init': LaunchConfiguration('y_init'),
            'z_init': LaunchConfiguration('z_init'),
            'R_init': LaunchConfiguration('R_init'),
            'P_init': LaunchConfiguration('P_init'),
            'Y_init': LaunchConfiguration('Y_init'),
            'rviz': LaunchConfiguration('ros_gz_rviz'),
            'log_level': 'error',
        }.items()
    )

    nav2_bringup_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_nav2_bringup'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'rviz': LaunchConfiguration('nav2_bringup_rviz'),
            'log_level': 'error',
        }.items(),
    )

    # bt_nav2_launch_description = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('bt_nav2'), 'launch', 'launch.py')
    #     ]),
    #     launch_arguments={
    #         'x_init': LaunchConfiguration('x_init'),
    #         'y_init': LaunchConfiguration('y_init'),
    #         'z_init': LaunchConfiguration('z_init'),
    #         'R_init': LaunchConfiguration('R_init'),
    #         'P_init': LaunchConfiguration('P_init'),
    #         'Y_init': LaunchConfiguration('Y_init'),
    #         'pose_cov_init': LaunchConfiguration('pose_cov_init'),
    #         'log_level': 'info',
    #     }.items(),
    # )
    

    return LaunchDescription([
        # Launch arguments
        x_init_launch_arg,
        y_init_launch_arg,
        z_init_launch_arg,
        R_init_launch_arg,
        P_init_launch_arg,
        Y_init_launch_arg,
        pose_cov_init_launch_arg,
        ros_gz_rviz_launch_arg,
        nav2_bringup_rviz_launch_arg,

        # Launch descriptions
        ros_gz_launch_description,
        nav2_bringup_launch_description,
        # bt_nav2_launch_description
    ])

