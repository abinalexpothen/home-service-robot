# home-service-robot

This is the final project in a series of projects within the Udacity Robotics Nanodegree Program.

## Summary
ROS gmapping package is used to first create a map of the Gazebo world using a turtlebot equipped with depth cameras. Subsequently, adaptive monte-carlo localization is used to localize the robot within this known map. The navigation package is then used to plan the path to a goal location. Two sets of goals are specified and a package named *pick_objects* is used to drive the turtlebot towards the pick-up goal and subsequently to the drop-off zone. Another package named *add_markers* subscribes to the turtlebot odometry and uses the known pick-up and drop-off locations to draw markers, which simulate the picking up and dropping off actions.

## Packages Used
The following packages are used in this project:

- **turtlebot_gazebo** is an official ROS package that simulates a robot with camera-based depth sensors in a gazebo world. The robot can be simulated within customs worlds by specifying the location of the file using the *world_file* parameter in the command-line call. It also provides the adaptive monte-carlo localization algorithm which is able to localize the robot within the known map of the world.

- **turtlebot_teleop** is an official ROS package that allows to drive the turtlebot around within the simulated world using keyboard inputs from the user. This package is used to drive the robot around while mapping.

- **gmapping** is a grid-based fast simultaneous localization and mapping (SLAM) algorithm that generates a two-dimensional occupany grid of the world from laser inputs. Since gmapping uses laser input, the depth camera information is transformed into a laser-like output and fed into the SLAM algorithm. Generating the map is the step that required immense patience as the camera had to be set in the field of view of multiple obstacles while tele-operating at the same time. The turn rate had to be reduced significantly so that the turtlebt could localize on the map correctly and slice together new locations appropriately within the already generated map. (Map generated with gmapping)

- **turtlebot_rviz_launcher** is an official 3D visualization package for ROS turtlebot. This provides a view_navigation.launch file which has certain pre-defined facilities allowing us to perform 2D navigation with ROS. It also displays the gmapping-generated 2D map, the trajectories and the obstacles within a grid. Special configurations could be loaded by specifying the appropriate *.rviz* configuration file using the *-d* option within the command-line.

- **pick_objects** is a package that commands the turtlebot to a pick-up goal location, wait for 5 seconds after reaching the goal, and then commands it to a drop-off goal location.

- **add_markers** simulates the action of picking up or dropping off a package through markers in rviz. The package does two types of action depending on the type of parameter that is passed to it
  - *_param:=addmarkers* This option results in a marker being displayed at the pickup location for 5 seconds, then the marker is deleted for 5 seconds, and then the marker is finally displayed at the drop-off location. 
  - *_param:=homeservice* The homeservice option results in the package subscribing to the odometry topic and it keeps track of whether it has reached the pickup or the dropoff zones. While starting the package with this option, the red marker is displayed on the pick-up location. When the robot odometry value is within this pickup zone at a specified tolerance, the code waits for 5 seconds and removes the marker, simulating a "pick-up" action. The code then check if the odometry values are close to the drop-off zone. Once the robot reaches the drop-off zone, the red marker is displayed, simulating a "drop-off".


**Turtlebot gazebo running abinshomeoffice.world**
![default_gzclient_camera(1)-2021-11-08T22_02_56 207014](https://user-images.githubusercontent.com/23329551/140854408-d83cce56-9d93-4237-a277-bbdbe59ab578.jpg)
