#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#define PI 3.14159265f

// Define a callback class
class PDListener
{
	public:
		void callback(const nav_msgs::Odometry::ConstPtr& msg_in);
		PDListener(ros::NodeHandle nh, ros::NodeHandle nhp);						// this is the constructor
	private:
		// Parameters
		float Pv, Pw, Dv, Dw;
		float v0, w0;
		// Position counter
		int pos;
		// Subscriber
		ros::Subscriber sub;
		ros::Publisher pub;
};
// Constructor
PDListener::PDListener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Get Parameters
	if (!nhp.getParam("Pv", Pv)) ROS_ERROR("pdControl: Could not find Pv parameter!");
	ROS_INFO("pdControl: Loaded Parameter\n Pv: %f", Pv);
	if (!nhp.getParam("Pw", Pw)) ROS_ERROR("pdControl: Could not find Pw parameter!");
	ROS_INFO("pdControl: Loaded Parameter\n Pw: %f", Pw);
	if (!nhp.getParam("Dv", Dv)) ROS_ERROR("pdControl: Could not find Dv parameter!");
	ROS_INFO("pdControl: Loaded Parameter\n Dv: %f", Dv);
	if (!nhp.getParam("Dw", Dw)) ROS_ERROR("pdControl: Could not find Dw parameter!");
	ROS_INFO("pdControl: Loaded Parameter\n Dw: %f", Dw);
	if (!nhp.getParam("v0", v0)) ROS_ERROR("randomWalk: Could not find v0 parameter!");
	ROS_INFO("randomWalk: Loaded Parameter\n v0: %f", v0);
	if (!nhp.getParam("w0", w0)) ROS_ERROR("randomWalk: Could not find w0 parameter!");
	ROS_INFO("randomWalk: Loaded Parameter\n w0: %f", w0);
	// Initialize subscriber
	sub = nh.subscribe("/odomKalmanFilter", 1, &PDListener::callback, this);
	// Initialize publisher
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	// Initialize variables
	pos = 0;
}
// Callback function to get the data from the topic
void PDListener::callback(const nav_msgs::Odometry::ConstPtr& msg_in)
{
	// Get message for pose and velocity
	nav_msgs::Odometry msg_pose = *msg_in;
	
	// Positions to visit
	float xT[20] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0,  6.0,  5.0,  4.0,  3.0,  3.0,  4.0,  4.0,  3.0,  2.0,  1.0,  0.0, -1.0, -1.0};
	float yT[20] = { 0.0, 0.0, 1.0, 1.0, 2.0, 1.0, 0.0, -1.0, -2.0, -1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0};

	// Check whether we are already near enough the desired position
	float ds = (xT[pos] - msg_pose.pose.pose.position.x)*(xT[pos] - msg_pose.pose.pose.position.x) + (yT[pos] - msg_pose.pose.pose.position.y)*(yT[pos] - msg_pose.pose.pose.position.y);
	if (ds < 0.3) {
		if (pos == 19) {
			pos = 0;
		}
		else {
			pos = pos + 1;
		}
	}
	
	// Transform quaternions to euler
	double qz = msg_pose.pose.pose.orientation.z;
	double qw = msg_pose.pose.pose.orientation.w;
	double siny_cosp = 2 * (qw * qz);
	double cosy_cosp = 1 - 2 * (qz * qz);
	double phi = std::atan2(siny_cosp,cosy_cosp);
	
	// Calculate Motor Commands, TODO: add D-part
	float e_x = xT[pos] - msg_pose.pose.pose.position.x;
	float e_y = yT[pos] - msg_pose.pose.pose.position.y;
	float u1 = Pv * (cosf(phi) * e_x + sinf(phi) * e_y);

	float phi_des = atan2f(e_y, e_x);
	if (phi > PI)
		phi = phi - 2 * PI;
	float dphi = phi_des - phi;
	if (dphi < -PI)
		dphi = 2*PI + dphi;
	if (dphi > PI)
		dphi = dphi - 2 * PI;
	float u2 = Pw * dphi;
	
	// Allocate motor control to message
	geometry_msgs::Twist msg_control;
	if (u1 > v0) {
		u1 = v0;
	}
	else if (u1 < 0) {
		u1 = 0;
	}
	if (u2 > w0) {
		u2 = w0;
	}
	else if (u2 < -w0) {
		u2 = -w0;
	}

	msg_control.linear.x = u1;
	msg_control.angular.z = u2;

	pub.publish(msg_control);
}


int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "decawaveControl");
  // Initialize node handle
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Initialize the subscriber to get the teleop data
  PDListener pdlistener(nh, nhp);
  // Start the loop
  ros::spin();
  return 0;
}
