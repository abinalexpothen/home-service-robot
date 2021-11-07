# home-service-robot

This is the final project in a series of projects within the Udacity Robotics Nanodegree Program.

Gmapping is used to first create a map of the Gazebo world and subsequently, adaptive monte-carlo localization is used to localize the robot within this known map. The navigation package is used to plan the path to a goal location. Two sets of goals are specified and a package named *pick_objects* is used to drive the turtlebot towards the pick-up goal and subsequently to the drop-off zone. Another package named *add_markers* subscribes to the turtlebot odometry and uses the known pick-up and drop-off locations to draw markers, which simulate the picking up and dropping off actions.
