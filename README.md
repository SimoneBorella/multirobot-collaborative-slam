# multirobot-collaborative-slam

Trutlebot3 simulation:
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Turtlebot keyboard command (along with running gazebo simulation):
	ros2 run turtlebot3_teleop teleop_keyboard
	
Turtlebot cartographer slam (along with running gazebo simulation):
	ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

Nav2 save generated map:
	ros2 run nav2_map_server map_saver_cli -f maps/my_map
	
Nav2 Turtlebot navigation (along with running gazebo simulation):
	ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=./src/slam_nav2_bringup/maps/map.yaml

	ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=./src/slam_nav2_bringup/maps/map.yaml

	Use 2d pose estimate and nav2 goal to set initial position and target position
	Use Waypoint





URDF to SDF:
    ign sdf -p /my_urdf.urdf > /my_sdf.sdf 





ign topic -l
ign topic -i --topic topic_name




ros2 topic pub /diff_drive/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}"



TF2 visualization (generate pdf):
	ros2 run tf2_tools view_frames



RUN SLAM TOOLBOX
	ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true


ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true /turtlebot3_waffle/scan:=/scan
