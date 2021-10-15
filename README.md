# PDE4603 Major Project (Thesis) Repository
## Overview
This repository contains files for the PDE4603 Major Project (Thesis). The topic of the work is "Development of an Automated Sorting System Based on Robot Vision". During the project, a robot vision system is being developed. It allows to determine objects' parameters such as colours and shapes. Based on this information, the system signals to the manipulator whether an object should be gripped or not. So, the robot arm automatically grips and sorts the objects with the relevant parameters. The project is being developed in the Robot Operating System (ROS). The simulation is done in Gazebo.  

The developer is Aleksei Korotchenko, a MSc Robotics student from Middlesex University London.

The project contains three folders. The first is MV_Python. It contains machine vision files. The program Test_Hue.py is auxiliary and is intended to determine the range of colour hues. MV_Python.py is the main program. It determines contours, shapes and colours of objects in the real-time video stream. After running it, the user should select the shape and colour of the object for gripping using the console. According to the entered data and the received video stream the programme determines whether the object should be gripped or not and displays this information. An example of how the program works, when the required object is a green rectangle, is shown below. This program is the basis of the machine vision in the project and it is also used in the Gazebo simulation. Programs can be run on a computer. Before running the programs, please check which video stream object is selected.  
  
<p align="center">
<img src="https://github.com/aleksei089/PDE4603_Major_Project/blob/main/ur5_ROS-Gazebo/media/Retesting%20with%20Real%20Objects%20Green%20Rectangle.jpg" width="405">  
  
The second folder is universal_robot. It contains model files of Universal Robots manipulators. One of them is used in the project, in Gazebo simulation. These files are taken from https://github.com/ros-industrial/universal_robot.  
  
The third folder is ur5_ROS-Gazebo. It contains the files that are needed to simulate robot vision in Gazebo. These include a model of the conveyor belt, camera, gripping objects, storage box, and vacuum gripper. All of these are needed to test and apply robot vision and object sorting in Gazebo simulation. The files were taken from https://github.com/lihuang3/ur5_ROS-Gazebo. In order to implement the created machine vision algorithm and test it in the simulation, some files were changed and some were created. For example, the vision and object spawning algorithms were changed and files for new gripping objects were created. Thus, the simulation is a sequential process. Initially, objects of various shapes and colours are spawned on a conveyor belt and moved towards the camera of manipulator. When an object comes into the camera's lens, the machine vision algorithm determines whether the item should be gripped or not. If the object fits the criteria, then the robot picks it up using a vacuum gripper and moves it into the box. An example of how the simulation works is shown in the illustrations below.  
  
<p align="center">
<img src="https://github.com/aleksei089/PDE4603_Major_Project/blob/main/ur5_ROS-Gazebo/media/RV_Blue_Circle_No_Grip.jpg" width="400">
<img src="https://github.com/aleksei089/PDE4603_Major_Project/blob/main/ur5_ROS-Gazebo/media/RV_Orange_Square_Grip.jpg" width="400">
<img src="https://github.com/aleksei089/PDE4603_Major_Project/blob/main/ur5_ROS-Gazebo/media/RV_Orange_Square_Put.jpg" width="400">  

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
After starting the simulation, the numbers, that correspond to the shape and colour of the required object for gripping, should be entered into the terminal. For shapes: 1 = Triangle, 2 = Square, 3 = Rectangle, 4 = Circle. For colours: 1 = Red, 2 = Orange, 3 = Yellow, 4 = Green, 5 = Blue,  
6 = Purple. After that, the video stream from the camera will be started and the simulation will work as planned.  
  
NOTE. This simulation will spawn the following sequence of four objects: a yellow square, a blue circle, a blue rectangle and a blue circle again.  However, it and the used objects can be changed. To do this, change or create new urdf files in ur5_ROS-Gazebo/urdf and change their path in initialize.launch in ur5_ROS-Gazebo/launch. Also if a sequence of more than four objects needs to be created, additions need to be made to blocks_spawner.cpp in ur5_ROS-Gazebo in the same way as for already added objects. The comments in the program should help in this situation.
