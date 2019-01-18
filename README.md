# Tutorial for ROS and Gazebo (under current development)
This is the code basis for ROS and Gazebo Tutorials taught at the University of Luebeck. 
A differential drive robot is built and equipped with a laser sensor, wheel odometry and an IMU. 
Up to the current state it cna be used by steering the robot in an apartment environment by simultanously mapping the environment using slam_gmapping.

## Requirements
* ROS (recommended: Kinetic), Installation Instructions under http://wiki.ros.org/kinetic/Installation/Ubuntu
* Gazebo (recommended: Version 7.0), comes with the ROS full version, otherwise http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0
* Additional ROS packages to install:
```bash
sudo apt-get install ros-kinetic-gmapping
sudo apt-get install ros-kinetic-teleop-twist-keyboard
```

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
roslaunch interfaces mapping.launch
```


