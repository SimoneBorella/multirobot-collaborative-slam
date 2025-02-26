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
	ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_map
	
	Use 2d pose estimate and nav2 goal to set initial position and target position
	Use Waypoint





URDF to SDF:
    ign sdf -p /my_urdf.urdf > /my_sdf.sdf 




Model topic commands:
<!--
  Try sending commands:
    gz topic -t "/model/diff_drive/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: -0.1}"
    ros2 topic pub /diff_drive/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}"
  Listen to odometry:
    gz topic -e -t /model/diff_drive/odometry
    ros2 topic echo /model/diff_drive/odometry
-->
