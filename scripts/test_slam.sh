 #!/bin/sh
 xterm -e " source devel/setup.bash " &
 xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/aordev/abinshomeoffice.world" &
 sleep 10
 xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch " &
 sleep 5
 xterm -e " rosrun gmapping slam_gmapping " &
 sleep 5
 xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch "
 sleep 5
