#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <math.h>

static ros::Publisher marker_pub;
static visualization_msgs::Marker marker;

static bool _PICKING_UP = false;
static bool _DROPPING_OFF = false;

static const float MARKER_SIZE = 0.25;

static const float POS_THRESH = 0.3;
static const float PICK_POS_X = 4.7;
static const float PICK_POS_Y = 3.5;
static const float PICK_POS_W = 1.0;
static const float DROP_POS_X = 2.8;
static const float DROP_POS_Y = -0.1;
static const float DROP_POS_W = -1.0;

void add_marker(){
  //visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker. This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "marker_id";
  marker.id = 0;

  // Set the marker type to CUBE
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  
  // Set the Marker pose and orientation
  marker.pose.position.x = PICK_POS_X;
  marker.pose.position.y = PICK_POS_Y;
  marker.pose.orientation.w = PICK_POS_W;

  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;

  if (_PICKING_UP)
  {
    marker.action = visualization_msgs::Marker::DELETE;
    ros::Duration(1.0).sleep();
    ROS_WARN_ONCE("<<---Robot picked-up object!--->>");    
  }
  if (_DROPPING_OFF)
  {
    marker.pose.position.x = DROP_POS_X;
    marker.pose.position.y = DROP_POS_Y;
    marker.pose.orientation.w = DROP_POS_W;
    marker.action = visualization_msgs::Marker::ADD;
    ros::Duration(1.0).sleep();
    ROS_WARN_ONCE("<<---Robot dropped-off object!--->>");
  }

  marker_pub.publish(marker);
}

void callback_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
  float robot_pos_x = msg->pose.pose.position.x;
  float robot_pos_y = msg->pose.pose.position.y;
  
  float pickup_dist = 0.0f;
  float dropoff_dist = 0.0f;

  if(!_PICKING_UP && !_DROPPING_OFF)
  {
    pickup_dist = pow( pow((PICK_POS_X - robot_pos_x),2) + pow((PICK_POS_Y - robot_pos_y),2), 0.5);
    ROS_INFO("Distance to pickup: ([%f])", pickup_dist);
    if( abs(pickup_dist) <= POS_THRESH )
    {
      ROS_WARN_ONCE("<<--- Robot reached pickup goal! --->>");
      _PICKING_UP = true;
      marker.action = visualization_msgs::Marker::DELETE;
    }
  }
  if(!_DROPPING_OFF && _PICKING_UP)
  {
    dropoff_dist = pow( pow((DROP_POS_X - robot_pos_x),2) + pow((DROP_POS_Y - robot_pos_y),2), 0.5);
    ROS_INFO("Distance to dropoff: ([%f])", dropoff_dist);
    if( abs(dropoff_dist) <= POS_THRESH )
    {
      ROS_WARN_ONCE("<<--- Robot reached dropoff goal! --->");
      _PICKING_UP = false;
      _DROPPING_OFF = true;
    }
  }
}

/*
int main(int argc, char** argv){
  ros::init(argc, argv, "add_marker");
  ros::NodeHandle n;
  ros::Rate r(1);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, callback_odom);

  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    ros::Duration(1.0).sleep();
  }
  add_marker();
  //add_marker(PICK_POS_X, PICK_POS_Y, PICK_POS_W);
  ros::spinOnce();

  return 0;
}
*/
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, callback_odom);

  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker. This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "marker_shapes";
  marker.id = 0;

  // Set the marker type to CUBE
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = MARKER_SIZE;
  marker.scale.y = MARKER_SIZE;
  marker.scale.z = MARKER_SIZE;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  
  // Set the Marker pose and orientation
  marker.pose.position.x = PICK_POS_X;
  marker.pose.position.y = PICK_POS_Y;
  marker.pose.orientation.w = PICK_POS_W;
    
  
  while (ros::ok())
  {
  
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

    if (_PICKING_UP)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      ros::Duration(1.0).sleep();
      ROS_WARN_ONCE("<<---Robot picked up object!--->>");
      
    }
    if (_DROPPING_OFF)
    {
      marker.pose.position.x = DROP_POS_X;
      marker.pose.position.y = DROP_POS_Y;
      marker.action = visualization_msgs::Marker::ADD;
      ros::Duration(1.0).sleep();
      ROS_WARN_ONCE("<<---Robot dropped off object!--->>");
    }
    marker_pub.publish(marker);

    ros::spinOnce();
  }

  return 0;
}