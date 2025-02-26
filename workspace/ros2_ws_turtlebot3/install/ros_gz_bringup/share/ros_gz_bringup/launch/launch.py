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

    world_file = 'turtlebot3_world.sdf'
    model_dir = 'turtlebot3'
    rviz_file = 'turtlebot3.rviz'
    x, y, z, R, P, Y = '-2.0', '-0.5', '0.0', '0.0', '0.0', '0.0'

    # Load model SDF file
    sdf_file  =  os.path.join(get_package_share_directory('ros_gz_description'), 'models', model_dir, 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_description = infp.read()

    # Launch descriptions

    ros_gz_sim_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                get_package_share_directory('ros_gz_gazebo'), 'worlds', world_file
            ])
        }.items(),
    )


    # Nodes

    ros_gz_sim_spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'waffle',
            '-file', os.path.join(get_package_share_directory('ros_gz_description'), 'models', model_dir, 'model.sdf'),
            '-x', x,   # X position
            '-y', y,   # Y position
            '-z', z,   # Z position
            '-R', R,   # Roll (rotation around x-axis)
            '-P', P,   # Pitch (rotation around y-axis)
            '-Y', Y   # Yaw (rotation around z-axis, in radians)
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )


    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('ros_gz_bringup'), 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    rviz_node = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(get_package_share_directory('ros_gz_bringup'), 'config', rviz_file)]
    )


    return LaunchDescription([
        # Launch descriptions
        ros_gz_sim_launch_description,

        # Nodes
        ros_gz_sim_spawn_robot_node,
        ros_gz_bridge_node,
        robot_state_publisher_node,
        rviz_node
    ])
