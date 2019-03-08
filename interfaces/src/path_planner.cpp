#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

// This is a real simple path planner for exploring a bounded area

// Define a callback class
class Listener
{
	public:
		void callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg_in);		// callback function to receive sensor measurements
		void publishNextPose();													// function to calculate motor commands
		Listener(ros::NodeHandle nh, ros::NodeHandle nhp);						// this is the constructor
	private:
		nav_msgs::OccupancyGrid map;
		// Subscriber
		ros::Subscriber sub;
		// Publisher
		ros::Publisher pub;
};
// Constructor
Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Initialize Subscriber
	sub = nh.subscribe("/map", 10, &Listener::callbackMap, this);
	// Initialize Publisher
	pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
}
// Function definitions
void Listener::callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg_in)
{
	map = *msg_in;
}
void Listener::publishNextPose() {
	// This is the planning algorithm
	geometry_msgs::PoseStamped msg_out;
	msg_out.pose.position.x = 6.0;
	msg_out.pose.position.y = 6.0;
	msg_out.pose.orientation.w = 1.0;
	msg_out.header.frame_id = "map";
	pub.publish(msg_out);
}

// The main function which starts the loop and initializes the ROS node
int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "path_planner");
  // Initialize node handle
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Initialize the subscriber to get the teleop data
  Listener listener(nh,nhp);
  // Loop rate with which we publish the goal state
  ros::Rate loop_rate(5);
  // Start the loop, but before just wait a few seconds until all sensors are ready to go
  ros::Duration(10).sleep();
  while (ros::ok())
  {
	// Subscribe using the callback functions
	ros::spinOnce();
	// Publish the next goal state
	listener.publishNextPose();
	// Sleep until rate is reached
    loop_rate.sleep();
  }
  return 0;
}
