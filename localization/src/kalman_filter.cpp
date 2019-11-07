#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"

#define PI 3.14159265f

// Define a callback class
class KFListener
{
	public:
		void callback(const nav_msgs::Odometry::ConstPtr& msg_in);					// Here we do the Kalman Filter
		KFListener(ros::NodeHandle nh, ros::NodeHandle nhp);						// this is the constructor
	private:
		ros::Subscriber subOdom;
		ros::Publisher pub;
};

// Constructor
KFListener::KFListener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Initialize subscriber
	subOdom = nh.subscribe("/odom", 1, &KFListener::callback, this);
	// Initialize publisher
	pub = nh.advertise<nav_msgs::Odometry>("/odomKalmanFilter", 1);
}

// Callback function to get the data from the topic
void KFListener::callback(const nav_msgs::Odometry::ConstPtr& msg_in)
{
	nav_msgs::Odometry msg_out;
	msg_out = *msg_in;
	pub.publish(msg_out);
}


int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "kalmanFilter");
  // Initialize node handle
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Initialize the subscriber to get the teleop data
  KFListener kflistener(nh, nhp);
  // Start the loop
  ros::spin();
  return 0;
}
