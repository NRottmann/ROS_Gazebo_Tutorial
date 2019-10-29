/*
 * This Plugin is a differential drive controller which takes commands 
 * over the /cmd_vel topic (if not stated othwerwise) and gives back 
 * odometry data over the /odom topic (if not stated otherwise)
*/

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "interfaces/Odometry.h"
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <random>

namespace gazebo
{
  class RobotControl : public ModelPlugin
  {
	  
	// Pointer to the model
    private: physics::ModelPtr model;
    
    // Pointer to joint
    private: physics::JointPtr jointLeft;
    private: physics::JointPtr jointRight;
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Handle for the gazebo ros node
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    // old joint positions
    private: double posLeftOld;
    private: double posRightOld;
    
    // Define Update Rate Variables
    private: double updateRate;
    private: double prevUpdateTime;
    
    // Define pose of the robot, starting by [0 0 0]
    double pose[3] = {0.0, 0.0, 0.0};
    
    // Radius of the wheels and axis distance
    double axisDistance;
    double radius;
    
    // The noise, TODO: Maybe change noise to Probabilistic Robotics odometry noise model
    /* std::vector<double> noise;
    double d_tmp = 0.0; */
    double noise;
    
    // Initialize Random Engine for Noise Generation
	std::default_random_engine generator;
    
    // ROS publisher
    ros::Publisher rosPub;
    
    // ROS subscriber
    ros::Subscriber rosSub;
	  
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {	
	  // Load parameters
	  std::string leftJoint;
	  if (!_sdf->HasElement("leftJoint")) {
		ROS_WARN("Missing parameter <leftJoint> in odometry_plugin, default to standard");
		leftJoint = "standard";
	  }
	  else {
	    leftJoint = _sdf->GetElement("leftJoint")->GetValue()->GetAsString();
	    ROS_INFO("odometry_plugin: leftJoint = %s", leftJoint.c_str());
	  }
	  std::string rightJoint;
	  if (!_sdf->HasElement("rightJoint")) {
		ROS_WARN("Missing parameter <rightJoint> in odometry_plugin, default to standard");
		rightJoint = "standard";
	  }
	  else {
	    rightJoint = _sdf->GetElement("rightJoint")->GetValue()->GetAsString();
	    ROS_INFO("odometry_plugin: rightJoint = %s", rightJoint.c_str());
	  }
	  double torque = 1000;
	  if (_sdf->HasElement("torque")) {
		torque = _sdf->Get<double>("torque");
		ROS_INFO("odoemtry_plugin: Set torque to %f", torque);
	  }
	  else
	    ROS_INFO("odoemtry_plugin: Default torque is %f", torque);
	  if (_sdf->HasElement("radius")) {
		radius = _sdf->Get<double>("radius");
		ROS_INFO("odoemtry_plugin: Set radius to %f", radius);
	  }
	  else
	  {
		radius = 0.1075;
	    ROS_INFO("odoemtry_plugin: Default radius is %f", radius);  
	  }
	  if (_sdf->HasElement("axisDistance")) {
		axisDistance = _sdf->Get<double>("axisDistance");
		ROS_INFO("odoemtry_plugin: Set axisDistance to %f", axisDistance);
	  }
	  else
	  {
		axisDistance = 0.1075;
	    ROS_INFO("odoemtry_plugin: Default axisDistance is %f", axisDistance);  
	  }
	  if (_sdf->HasElement("updateRate")) {
		updateRate = _sdf->Get<double>("updateRate");
		ROS_INFO("odoemtry_plugin: Set updateRate to %f", updateRate);
	  }
	  else
	  {
		updateRate = 0.05;
	    ROS_INFO("odoemtry_plugin: Default updateRate is %f", updateRate);  
	  }
	  std::string cmdTopic;
	  if (_sdf->HasElement("cmdTopic")) {
		cmdTopic = _sdf->GetElement("cmdTopic")->GetValue()->GetAsString();
	    ROS_INFO("odometry_plugin: cmdTopic = %s", cmdTopic.c_str());
	  }
	  else {
	    cmdTopic = "/cmd_vel";
	    ROS_INFO("odometry_plugin: Default cmdTopic is %s", cmdTopic.c_str());
	  }
	  std::string odomTopic;
	  if (_sdf->HasElement("odomTopic")) {
		odomTopic = _sdf->GetElement("odomTopic")->GetValue()->GetAsString();
	    ROS_INFO("odometry_plugin: odomTopic = %s", odomTopic.c_str());
	  }
	  else {
	    odomTopic = "/odom";
	    ROS_INFO("odometry_plugin: Default cmdTopic is %s", cmdTopic.c_str());
	  }
	  if (_sdf->HasElement("noise")) {
		noise = _sdf->Get<double>("noise");
		ROS_INFO("odoemtry_plugin: Set noise to %f", noise);
	  }
	  else
	  {
		noise = 0.1;
	    ROS_INFO("odoemtry_plugin: Default noise is %f", noise);  
	  }
	  /* if (_sdf->HasElement("noise")) {
		std::string noise_string = _sdf->GetElement("noise")->GetValue()->GetAsString();
		std::stringstream ss_noise_string(noise_string);
		while(ss_noise_string >> d_tmp)
			noise.push_back(d_tmp);
		ROS_INFO("odoemtry_plugin: Set noise to %f %f %f %f", noise[0], noise[1], noise[2], noise[3]);
	  }
	  else
	  {
		for(int i=0; i<4; i++) {
			noise[i] = 0.1;
		}
	    ROS_INFO("odoemtry_plugin: Default noise to %f %f %f %f", noise[0], noise[1], noise[2], noise[3]); 
	  } */
	  
	  // Initialize values
	  posLeftOld = 0;
	  posRightOld = 0;
	  
      // Store the pointer to the model
      this->model = _parent;
      
      // Get the joint
      this->jointLeft = this->model->GetJoint(leftJoint);
      this->jointRight = this->model->GetJoint(rightJoint);
      
      // Configure joint motor, TODO: Get realistic Moment M = (U*I)/(2*pi*n)
      this->jointLeft->SetParam("fmax", 0, torque);
      this->jointRight->SetParam("fmax", 0,torque);
      
      // initialize the prevUpdateTime
      this->prevUpdateTime = common::Time::GetWallTime().Double();

	  // Make sure the ROS node for Gazebo has already been initialized  
	  if (!ros::isInitialized())
      {
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		return;
      }
      
      // Reset the ros node name and initialize subscriber and publisher
      this->rosNode.reset(new ros::NodeHandle(""));
      rosPub = this->rosNode->advertise<interfaces::Odometry>(odomTopic, 10);
      rosSub = this->rosNode->subscribe(cmdTopic, 10, &RobotControl::ROSCallback, this);
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RobotControl::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {  
	  double time_tmp = common::Time::GetWallTime().Double();
	  if (time_tmp - this->prevUpdateTime < this->updateRate)
		return;
		
	  // Get position of the wheels
	  double posLeft = this->jointLeft->GetAngle(0).Radian();
	  double posRight = this->jointRight->GetAngle(0).Radian();

      // Publish to rostopic odomTopic with noisy data
      interfaces::Odometry msg_out;
      double l_R = radius * (posRight - posRightOld);
      double l_L = radius * (posLeft - posLeftOld);
      
      std::normal_distribution<double> gaussianDistribution(0.0,noise);
      msg_out.l_R = l_R * (1 + gaussianDistribution(generator));
      msg_out.l_L = l_L * (1 + gaussianDistribution(generator));
      
      posRightOld = posRight;
      posLeftOld = posLeft;
      this->rosPub.publish(msg_out);   
      
      this->prevUpdateTime = time_tmp;
    }
    
    public: void ROSCallback(const geometry_msgs::Twist& msg)
	{
	  // Get desired velocities, TODO: we should also add here noise for a realistic model
	  double velRight = (msg.linear.x + axisDistance * msg.angular.z) / radius;
      double velLeft = (msg.linear.x - axisDistance * msg.angular.z) / radius;
      // Set velocities
	  this->jointRight->SetParam("vel", 0, velRight);
	  this->jointLeft->SetParam("vel", 0, velLeft);
	}

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RobotControl)
}
