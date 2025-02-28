import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node


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

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
    )


    # Environment variables

    gz_resource_path_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(get_package_share_directory('ros_gz_description'), 'models'))

    # Launch descriptions

    ros_gz_sim_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                get_package_share_directory('ros_gz_gazebo'), 'worlds', 'turtlebot3_world.sdf'
            ]),
            'on_exit_shutdown': 'True',
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )


    # Nodes

    create_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'turtlebot3_waffle',
            '-file', os.path.join(get_package_share_directory('ros_gz_description'), 'models', 'turtlebot3', 'model.sdf'),
            '-x', LaunchConfiguration('x_init'),
            '-y', LaunchConfiguration('y_init'),
            '-z', LaunchConfiguration('z_init'),
            '-R', LaunchConfiguration('R_init'),
            '-P', LaunchConfiguration('P_init'),
            '-Y', LaunchConfiguration('Y_init'),
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('ros_gz_description'), 'models', 'turtlebot3', 'model.sdf')])
        }],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )


    ros_gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('ros_gz_bringup'), 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', os.path.join(get_package_share_directory('ros_gz_bringup'), 'config', 'turtlebot3.rviz'),
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
        # Environment variables
        gz_resource_path_env,

        # Launch arguments
        x_init_launch_arg,
        y_init_launch_arg,
        z_init_launch_arg,
        R_init_launch_arg,
        P_init_launch_arg,
        Y_init_launch_arg,
        rviz_launch_arg,
        log_level_launch_arg,

        # Launch descriptions
        ros_gz_sim_launch_description,

        # Nodes
        create_robot_node,
        ros_gazebo_bridge_node,
        robot_state_publisher_node,
        rviz_node
    ])
