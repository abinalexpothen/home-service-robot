 #!/bin/sh
 xterm -e " source devel/setup.bash " &
 xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/aordev/workspace/home-service-robot/src/world/abinshomeoffice.world " &
 sleep 10
 xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch " &
 sleep 5
 xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch "
