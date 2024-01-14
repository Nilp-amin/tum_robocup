# TUM RoboCup@Home
Repo for the RoboCup@Home 2023 project at TUM. This repository contains a variety of packages for the [Clean Up task.](https://athome.robocup.org/wp-content/uploads/2022_rulebook.pdf) 

<!-- # Team members
1. Nilp (Matrikel-Nr: 03784634) 
2. Niklas (Matrikel-Nr: 03714848)
3. Sahil (Matrikel-Nr: ) -->

# Requirements
To be able to use this repository the below requirements have to be met.
1. The *hsrb development packages* needs to be setup and built. This can be done by following the instructions at [hsrb](https://docs.hsr.io/hsrb_user_manual_en/howto/pc_install.html).

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
4. Setup [gpd_ros](https://github.com/atenpas/gpd_ros) by using the below commands.
```
cd ~
git clone https://github.com/atenpas/gpd
cd gpd
mkdir build && cd build
cmake ..
sudo make -j install
cd ~/catkin_ws/src
git clone https://github.com/atenpas/gpd_ros
```
5. Copy the [hsrb_wrs_gazebo](https://gitlab.lrz.de/robocup-home-ics/tutorials/-/wikis/uploads/T5_fengyi/hsrb_wrs_gazebo.zip) folder into the *catkin_ws* using the below commands after unzipping.
```
mv <path-to-extracted-hsrb_wrs_gazebo-folder> ~/catkin_ws/src
```
6. Copy the [hsrb_moveit](https://gitlab.lrz.de/robocup-home-ics/tutorials/-/wikis/uploads/T5_fengyi/hsrb_moveit.zip) folder into the *catkin_ws* using the below commands after unzipping.
```
mv <path-to-extracted-hsrb_moveit-folder> ~/catkin_ws/src
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

