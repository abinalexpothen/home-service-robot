#!/bin/sh

cd "$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"/../

world_file_location="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"/world/abinshomeoffice.world

cd "$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"/../

setup_file_location="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"/devel/setup.bash

xterm -e " source $setup_file_location; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$world_file_location " &
sleep 10
xterm -e " source $setup_file_location; roslaunch turtlebot_teleop keyboard_teleop.launch " &
sleep 5
xterm -e " source $setup_file_location; rosrun gmapping slam_gmapping " &
sleep 5
xterm -e " source $setup_file_location; roslaunch turtlebot_rviz_launchers view_navigation.launch "
