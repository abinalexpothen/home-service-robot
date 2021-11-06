#!/bin/sh

cd "$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"/../

world_file_location="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"/world/abinshomeoffice.world
map_file_location="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"/map/abinshomeoffice.yaml

cd "$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"/../

setup_file_location="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"/devel/setup.bash

xterm -e " source $setup_file_location; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$world_file_location " &
sleep 10
xterm -e " source $setup_file_location; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$map_file_location initial_pose_a:=-1.571" &
sleep 5
xterm -e " source $setup_file_location; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 8
xterm -e " source $setup_file_location; rosrun pick_objects pick_objects"