import os
import yaml
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # Launch arguments

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )

    bag_record_launch_arg = DeclareLaunchArgument(
        'bag_record',
        default_value='False',
        description='Enable ros2 bag recording'
    )

    rviz_nav2_launch_arg = DeclareLaunchArgument(
        'rviz_nav2',
        default_value='False'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )



    ld.add_action(namespace_launch_arg)
    ld.add_action(bag_record_launch_arg)
    ld.add_action(rviz_nav2_launch_arg)
    ld.add_action(log_level_launch_arg)


    timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    bag_name = f'./bag_records/bag_{timestamp}'

    topics_to_record = [
        '/robot_12/frontier',
        '/robot_12/frontier_centroids',
        '/robot_12/global_costmap/costmap',
        '/robot_12/global_costmap/costmap_raw',
        '/robot_12/global_costmap/costmap_updates',
        '/robot_12/global_costmap/footprint',
        '/robot_12/global_costmap/global_costmap/transition_event',
        '/robot_12/global_costmap/published_footprint',
        '/robot_12/global_costmap/scan',
        '/robot_12/goal_pose',
        '/robot_12/imu',
        '/robot_12/joint_states',
        '/robot_12/landmarks',
        '/robot_12/landmarks_marker',
        '/robot_12/local_costmap/costmap',
        '/robot_12/local_costmap/costmap_raw',
        '/robot_12/local_costmap/costmap_updates',
        '/robot_12/local_costmap/footprint',
        '/robot_12/local_costmap/local_costmap/transition_event',
        '/robot_12/local_costmap/published_footprint',
        '/robot_12/local_costmap/scan',
        '/robot_12/local_plan',
        '/robot_12/map',
        '/robot_12/marker',
        # '/robot_12/oak/rgb_landmarks/image_raw',
        # '/robot_12/oak/rgb_landmarks/image_raw/compressed',
        # '/robot_12/oak/rgb_landmarks/image_raw/compressedDepth',
        # '/robot_12/oak/rgb_landmarks/image_raw/theora',
        '/robot_12/odom',
        '/robot_12/plan',
        '/robot_12/planner_server/transition_event',
        '/robot_12/preempt_teleop',
        '/robot_12/received_global_plan',
        '/robot_12/robot_description',
        '/robot_12/scan',
        '/robot_12/sensor_state',
        '/robot_12/speed_limit',
        '/robot_12/tf',
        '/robot_12/tf_static',
        '/robot_12/transformed_global_plan',
        '/robot_12/velocity_smoother/transition_event',
        '/robot_12/waypoint_follower/transition_event'
    ]

    bag_record_execute_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_name] + topics_to_record,
        output='screen',
        condition=IfCondition(LaunchConfiguration('bag_record')),
    )

    ld.add_action(bag_record_execute_process)


    # Launch descriptions

    turtlebot3_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_wrapper'), 'launch', 'robot.launch.py')
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'log_level': LaunchConfiguration('log_level'),
            'cam_pos_z': "0.235"
        }.items(),
    )

    sensor_processing_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sensor_processing'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    slam_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    nav2_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_launch'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'slam_toolbox': 'False',
            'rviz': LaunchConfiguration('rviz_nav2'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    ld.add_action(turtlebot3_launch_description)
    ld.add_action(sensor_processing_launch_description)
    ld.add_action(slam_launch_description)
    ld.add_action(nav2_launch_description)
        
    return ld

