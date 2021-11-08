#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <vector>

ros::Publisher marker_pub;
visualization_msgs::Marker marker;

double displacement_tolerance = 0.25;
std::vector<double> PICKUP_LOC = {3.0, 1.0};
std::vector<double> DROPOFF_LOC = {2.5, -3.0};

// robot state machine
enum robot_state
{
  NONE,
  GOING_TO_PICKUP_ZONE,
  AT_PICKUP_ZONE,
  GOING_TO_DROPOFF_ZONE,
  AT_DROPOFF_ZONE,
};

robot_state state = GOING_TO_PICKUP_ZONE;

void process_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  switch(state)
  {
    case GOING_TO_PICKUP_ZONE:
    {
      double displacement_pickup = sqrt(pow((x-PICKUP_LOC[0]),2)+pow((y-PICKUP_LOC[1]),2));
      if (displacement_pickup < displacement_tolerance)
      {
        ROS_INFO("Reached pickup zone");
        state = AT_PICKUP_ZONE;          
      }
      break;
    }

    case AT_PICKUP_ZONE:
    {
      ROS_INFO("Picking up package ...");
      sleep(5);
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      state = GOING_TO_DROPOFF_ZONE;
      break;
    }

    case GOING_TO_DROPOFF_ZONE:
    {
      double displacement_dropoff = sqrt(pow((x-DROPOFF_LOC[0]),2)+pow((y-DROPOFF_LOC[1]),2));
      if (displacement_dropoff < displacement_tolerance)
      {
        ROS_INFO("Reached dropoff zone");
        state = AT_DROPOFF_ZONE;          
      }
      break;
    }

    case AT_DROPOFF_ZONE:
    {
      ROS_INFO("Dropping off package ...");
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = DROPOFF_LOC[0];
      marker.pose.position.y = DROPOFF_LOC[1];
      marker_pub.publish(marker);
      state = NONE;
      break;
    }

    case NONE:
    {
      break;
    }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n("~");

  std::string cmdparam;
  n.getParam("param", cmdparam);

  marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;
  
  // display marker at pick-up location and wait for 5 seconds
  marker.action = visualization_msgs::Marker::ADD;
  
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = PICKUP_LOC[0];
  marker.pose.position.y = PICKUP_LOC[1];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  if (cmdparam.compare("addmarkers") == 0)
  {
    // if executing add_markers.sh
    ROS_INFO("Called from add_markers.sh");

    marker_pub.publish(marker);
  
    sleep(5);
  
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
  
    sleep(5);
    
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = DROPOFF_LOC[0];
    marker.pose.position.y = DROPOFF_LOC[1];
    marker_pub.publish(marker);

    sleep(5);
  }
  else if (cmdparam.compare("homeservice") == 0)
  {
    // if executing home_service.sh
    ROS_INFO("Called from home_service.sh");

    marker_pub.publish(marker);
    
    ros::Subscriber sub = n.subscribe("/odom", 1000, process_odometry);
    ros::spin();
  }
  else
  {
    ROS_INFO("Unknown call to add_markers");
  }
 }