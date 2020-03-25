#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define PI 3.14159265f

// Define a callback class for the Kalman Filter
class KFListener
{
	public:
		// The Callback function to subscribe to the odomtery topic (/odom)
		void callbackOdom(const nav_msgs::Odometry::ConstPtr& msgIn);
		// The Callback function to subscribe to the imu topic (/imu)
		void callbackImu(const sensor_msgs::Imu::ConstPtr& msgIn);
		// The Kalman Filter function
		geometry_msgs::PoseWithCovarianceStamped kalmanFilter();
		// The Constructor of the class, here we will initialize the subscriber and publisher among other things
		KFListener(ros::NodeHandle nh, ros::NodeHandle nhp);						
	private:
		// Here we have to define instances of our publisher and subscriber
		ros::Subscriber subOdom;
		ros::Subscriber subImu;
		ros::Publisher pubPose;
		// Here we have to define other variables, for example to store the recent measurements
		nav_msgs::Odometry msgOdom;
		sensor_msgs::Imu msgImu;

};

// Define the constructor of the class, thus we have to initialize all subscriber and publisher
KFListener::KFListener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Initialize subscriber
	subOdom = nh.subscribe("/odom", 20, &KFListener::callbackOdom, this);
	subImu = nh.subscribe("/imu", 20, &KFListener::callbackImu, this);
	// Initialize publisher
	pubPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose", 20);
}

// Definition of the callback functions
void KFListener::callbackOdom(const nav_msgs::Odometry::ConstPtr& msgIn)
{
	msgOdom = *msgIn;
}
void KFListener::callbackImu(const sensor_msgs::Imu::ConstPtr& msgIn)
{
	msgImu = *msgIn;
}

// Here we would define a Kalman Filter function
geometry_msgs::PoseWithCovarianceStamped KFListener::kalmanFilter() {
	geometry_msgs::PoseWithCovarianceStamped msgOut;	
	/*
	*	Some Code for the Kalman Filter
	*/
	pubPose.publish(msgOut);
}

int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "kalmanFilter");
  // Initialize node handle
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Get instance of the class
  KFListener kflistener(nh, nhp);
  // Start the loop
  ros::spin();
  return 0;
}
