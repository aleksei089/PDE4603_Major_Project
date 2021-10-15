# PDE4603 Major Project (Thesis) Repository
## Overview
This repository contains the program files for the PDE4603 Major Project (Thesis).

The developer is Aleksei Korotchenko, a MSc Robotics student from Middlesex University London.

The project presents ...

## Getting Started Commands (Installation)
1. Create workspace:
```
mkdir -p ~/RV_Project_ws/src
```
2. Clone Robot Vision Project repository:
```
cd ~/RV_Project_ws/src
git clone https://github.com/aleksei089/PDE4603_Major_Project .
```
3. Make:
```
cd ~/RV_Project_ws/
catkin_make && source devel/setup.bash
```
If catkin_make is unsuccessful, try to install all necessary packages using the "sudo apt-get install" command.  
Also, you can try catkin_make a few times. All this should help.
## Usage
1. Launch Gazebo simulation:
```
roslaunch ur5_notebook initialize.launch
```
After starting the simulation, the numbers, that correspond to the shape and colour of the required object for gripping, should be entered into the terminal. For shapes: 1 = Triangle, 2 = Square, 3 = Rectangle, 4 = Circle. For colours: 1 = Red, 2 = Orange, 3 = Yellow, 4 = Green, 5 = Blue, 6 =  
Purple. After that, the video stream from the camera will be started and the simulation will work as planned.  
  
NOTE. This simulation will spawn the following sequence of four objects: a yellow square, a blue circle, a blue rectangle and a blue circle again.  However, it and the used objects can be changed. To do this, change or create new urdf files in ur5_ROS-Gazebo/urdf and change their path in initialize.launch in ur5_ROS-Gazebo/launch. Also if a sequence of more than four objects needs to be created, additions need to be made to blocks_spawner.cpp in ur5_ROS-Gazebo in the same way as for already added objects. The comments in the program should help in this situation.
