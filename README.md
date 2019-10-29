# Tutorial for ROS and Gazebo
This is the code basis for ROS and Gazebo Tutorials and Assignments. We use a simulated differential drive robot equipped with
* IMU
* Odometer
* Temperature Sensor

## Requirements
* ROS (recommended: Kinetic), Installation Instructions under http://wiki.ros.org/kinetic/Installation/Ubuntu
* Gazebo (recommended: Version 7.0), comes with the ROS full version, otherwise http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0

## How to:
* Start by creating a catkin workspace folder, downloading the git repository and compiling the code
```bash
mkdir -p ~/tutorial_ws/src
cd ~/tutorial_ws/src 
git clone https://github.com/NRottmann/ROS_Gazebo_Tutorial.git
cd  ..
catkin_make
```
* Start the simulation
```bash
cd ~/tutorial_ws
source devel/setup.bash
roslaunch simulation_environment apartment.launch
```
* The Gazebo environment should have opened and something similar to the image below should have appeared:
![Image of Gazebo](images/GazeboEnvironment.png "Gazebo Simulation Environment")

* Now you can move the robot around by opening a new terminal and typing
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Assignments:
Here we explain shortly the Assignments and what has to be changed or added to pass the Assignments.

* Assignment 01 - Kalman Filter
