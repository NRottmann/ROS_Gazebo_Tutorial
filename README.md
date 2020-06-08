# Tutorial for ROS and Gazebo
This is the code basis for ROS and Gazebo Tutorials and Assignments. We use a simulated differential drive robot equipped with
* IMU
* Odometer
* LiDAR
* Camera </br>
Check out the video tutorials [here](https://www.youtube.com/watch?list=PLlcq6PMeufrApLSWZR73ivGDjTxayA_Ss&v=brDKAweq_IM&feature=emb_title) for getting a detailed introduction into ROS and Gazebo.

## Requirements
* ROS Kinetic (Ubuntu 16.04) or Melodic (Ubuntu 18.04), Installation Instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Gazebo (recommended: Version 7.0), comes with the ROS full desktop version, otherwise Installation Instructions can be found [here](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0)
* (optional) ROS-QTC-Plugin for using QT Creator as IDE, Installation Instructions can be found [here](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/Improve-ROS-Qt-Creator-Plugin-Developers-ONLY.html)

## Additional Required Packages (installation instructions are given for ROS Melodic)
* [gmapping](http://wiki.ros.org/gmapping)
```bash
sudo apt-get install ros-melodic-gmapping
```
* [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)
```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
```
* [map_server](http://wiki.ros.org/map_server)
```bash
sudo apt-get install ros-melodic-map-server
```
* [amcl](http://wiki.ros.org/amcl)
```bash
sudo apt-get install ros-melodic-amcl
```
* [move_base](http://wiki.ros.org/move_base)
```bash
sudo apt-get install ros-melodic-move-base
```
* [global_planner](http://wiki.ros.org/global_planner)
```bash
sudo apt-get install ros-melodic-global-planner
```
* [teb_local_planner](http://wiki.ros.org/teb_local_planner)
```bash
sudo apt-get install ros-melodic-teb-local-planner
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
roslaunch simulation_environment apartment.launch
```
* The Gazebo environment should have opened and something similar to the image below should have appeared:
![Image of Gazebo](images/GazeboEnvironment.png "Gazebo Simulation Environment")

* Now you can move the robot around by opening a new terminal and typing
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Important Topics (Message Types) provided by the Simulation Environment
* Published
  * /camera/image_raw 	(sensor_msgs/Image)
  * /imu		(sensor_msgs/Imu)
  * /odom		(nav_msgs/Odometry)
  * /scan		(sensor_msgs/LaserScan)
* Subsrcibed
  * /cmd_vel		(geoemtry_msgs/Twist)

## Assignment 01:
1. Generate a Map of the Environment
  * Start the simulation environment and gampping (hint: have a look into the mapping.launch file)
  * Drive the robot around using the teleop_twist_keyboard until you the map is sufficient accurate (hint: you can have a look onto the map by using rviz)
  ```bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```
  * Save the map using map_server
  ```bash
  rosrun map_server map_saver -f myMapName
  ```

2. Find the Person in the Building (the robot will start at a random position outside the building)
  * Create a ROS Node which enables the robot to search in the apartment environment for the missing person (The person will be somewhere placed). After finding the missing person, mark the position of the missing person using visualization_msgs::Marker which can be displayed in rviz. The topic has to be name missing_person. An example on how to do such a visualization message can be found [here](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines).
  * For localization, you can use the amcl (Particle Filter) package (example: localization.launch)
  * For the navigation, you can use move_base, together with local and global path planner (example: navigation.launch). The navigation requires of course localization provided by tthe amcl package.
  * For moving the robot around, you can publish poses to the robot to the topic /move_base_simple/goal. The move_base package will do thee planning (if correctly configured) using a global and a local path planner.
  * The missing person can be detected by using the person_detector (example: detection.launch). A new topic /person_detector will appear which publishes the message pal_person_detector_opencv/Detections2d. This message contains information about detected person in the camera image. For more information about the person detector, we refer to the [ros wiki](http://wiki.ros.org/Robots/TIAGo/Tutorials/PersonDetection). For simplicity, we included the required parts of the pal repository into our tutorial repository.

### Submission

* Please fork the repository to your own GitHub profile, duplicate it and set the duplicate to private. Add me, NRottmann, as a contributor such that I can donwload it later. Then start editing your duplicat repository such that the assignment is full filled. Finally, send an email to Nils.Rottmann (at) rob.uni-luebeck.de with the concern Rescue_Assignment_01 with simply the link to your forked GitHub repository.
* Optional (if you do not have an own GitHub Account): Check out a new branch and solve the assignment in this branch. Afterwards, push the branch to this repository.  Finally, send an email to Nils.Rottmann (at) rob.uni-luebeck.de with the concern Rescue_Assignment_01 with simply the name of the pushed branch.
* Add instructions for installation and usage of your repository below under the headline Participant Instructions.

### Remarks

* There are a lot of good manuals on how ROS and Gazebo work, besides the provided video tutorials.
* The [ROS wiki](http://wiki.ros.org/) is quite extensive and provides good explanations for all here used packages.
* If you have a questions with regard to ROS or Gazebo, remember: You are probably not the first one who asked this. Thus, a simple google search might already solve your problem.
* There are a lot of good communities which provide excellent help with regard to ROS and Gazebo, e.g. Q&A at the ROS wiki, Q&A at the Gazebo wiki, stackoverflow, etc. Do not hesitate to ask your own question but first check whether there exists already a similar thread.
* Finally, if you tried the above and you still have an unanswered question, then feel free to ask us via Mail, Nils.Rottmann (at) rob.uni-luebeck.de

## Participant Instructions

...



## Assignment 02:

In this assignment, you have to let the robot automatically detect two fire sources. Therefore, the robot has a temperature sensor on board which publishes to the 

```
/temperature (sensor_msgs/Temperature)
```

topic. For running the scenario enter

```
roslaunch simulation_environment temperature.launch
```

The scenario is a free world scenario, thus with only a ground plane and no other boundaries at all. This enables fast simulations. To successfully complete the assignment, you have to:

* let the robot autonomously find both fire sources

* after finding each fire source, publish a self-designed message consisting of  the ROS messages

  * sensor_msgs/Temeperature
  * geometry_msgs/Pose

  to give out the fire source location together with the temperature

### Submission

* Please fork the repository to your own GitHub profile, duplicate it and set the duplicate to private. Add me, NRottmann, as a contributor such that I can donwload it later. Then start editing your duplicat repository such that the assignment is full filled. Finally, send an email to Nils.Rottmann (at) rob.uni-luebeck.de with the concern Rescue_Assignment_02 with simply the link to your forked GitHub repository. 
* Optional (if you do not have an own GitHub Account): Check out a new branch and solve the assignment in this branch. Afterwards, push the branch to this repository.  Finally, send an email to Nils.Rottmann (at) rob.uni-luebeck.de with the concern Rescue_Assignment_02 with simply the name of the pushed branch.
* Add instructions for installation and usage of your repository below under the headline Participant Instructions.

## Participant Instructions

...
