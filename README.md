# TUM RoboCup@Home
Repo for the RoboCup@Home 2023 project at TUM. This repository contains a variety of packages for the [Clean Up task.](https://athome.robocup.org/wp-content/uploads/2022_rulebook.pdf) 

<!-- # Team members
1. Nilp (Matrikel-Nr: 03784634) 
2. Nikolas (Matrikel-Nr:)
3. Sahil (Matrikel-Nr: ) -->

# Requirements
To be able to use this repository the below requirements have to be met.
1. The *tiago_public_ws* repository needs to be setup and built. This can be done by cloning the repository [locally](https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS) or setting up the [docker](https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/Installing_Tiago_tutorial_docker).

2. A seperate *catkin_ws* repository needs to be setup. This can be done by following the instructions below:
```
cd ~
mkdir -p catkin_ws/src && cd catkin_ws
catkin build
```
3. Clone [darknet_ros](https://github.com/leggedrobotics/darknet_ros) into the *catkin_ws* using the below commands.
```
cd ~/catkin_ws/src
git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
cd ..
```
4. Clone the [tmc_wrs_gazebo_world](https://gitlab.lrz.de/robocup-home-ics/tutorials/-/wikis/uploads/T5_fengyi/tmc_wrs_gazebo_world.zip) into the *catkin_ws* using the below commands after unzipping.
```
mv <path-to-extracted-tmc_wrs_gazebo_world-folder> ~/catkin_ws/src
```

# Set Up
Clone this repository into the *catkin_ws* that was created in the requirements step as follows:
```
cd ~/catkin_ws/src
git clone git@github.com:Nilp-amin/tum_robocup.git
cd ..
catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-8
catkin build
```
# Development
When working on your own tasks make sure to work on your own branch to avoid conflicts with another developer's codebase.

## Tasks
1. Nilp - Object Manipulation 
2. Nikolas - Object detection
3. Sahil - Task planning

