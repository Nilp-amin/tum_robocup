# TUM RoboCup@Home
Repo for the RoboCup@Home 2023 project at TUM. This repository contains a variety of packages for the [Clean Up task.](https://athome.robocup.org/wp-content/uploads/2022_rulebook.pdf) 

<!-- # Team members
1. Nilp (Matrikel-Nr: 03784634) 
2. Niklas (Matrikel-Nr: 03714848)
3. Sahil (Matrikel-Nr: 03699104) -->

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
git clone git@github.com:atenpas/gpd.git 
cd gpd
mkdir build && cd build
cmake ..
sudo make -j install
cd ~/catkin_ws/src
git clone git@github.com:atenpas/gpd_ros.git 
```
5. Copy the [hsrb_wrs_gazebo](https://gitlab.lrz.de/robocup-home-ics/tutorials/-/wikis/uploads/T5_fengyi/hsrb_wrs_gazebo.zip) folder into the *catkin_ws* using the below commands after unzipping.
```
mv <path-to-extracted-hsrb_wrs_gazebo-folder> ~/catkin_ws/src
```
# Set Up
Clone this repository into the *catkin_ws* that was created in the requirements step as follows:
```
cd ~/catkin_ws/src
git clone git@github.com:Nilp-amin/tum_robocup.git
```
Download the custom YOLO [weights](https://drive.google.com/file/d/1yZSaTLOWRaDS9rBSHwXwhvXUmixQYYU1/view?usp=sharing) and put them into the *catkin_ws* using the below commands:

```
cd ~/catkin_ws/src/tum_robocup/object_detection_world/config
mv <location-of-weights-file> .
```

Then build the entire workspace. Note, that it will fail initially since custom ROS messages are used. Hence, keep building until no errors remain.

```
cd ~/catkin_ws 
catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-8
catkin build
```

# Running
Assuming all the above steps have been completed, follow the below steps to run the entire system. It is also assumed that the HSRB robot is powered on and is connected to the same network as the developer PC which will run this workspace. Open six terminal windows which will be referred to A, B, C, D, E, and F henceforth.

1. In all terminals run the below commands:
```
hsrb_mode
cd ~/catkin_ws
. ./devel/setup.bash
``` 
2. In terminal A, launch the HSRB moveit controllers using the below command:
```
roslaunch hsrb_moveit_config hsrb_demo_with_controller.launch
```
3. In terminal B, launch the Computer Vision packages of the workspace with the below command:
```
roslaunch object_detection_world object_cv.launch
```
4. In terminal C, launch the GPD ROS node using the below command:
```
roslaunch object_manipulation gpd_ros.launch
```
5. In terminal D, launch the interface node between GPD and Moveit using the below command:
```
roslaunch object_manipulation object_manipulation.launch
```
6. In terminal E, launch the AMCL localisation node using the below command:
```
roslaunch hsrb_amcl acml_hsrb_realworld.launch
```
7. In terminal F, launch the FSM using the below command:
```
roslaunch task_planning task_manager.launch
```
8. Optionally, if you would like to visualise the system with RViZ the below commands can be run after opening a new terminal window:
```
hsrb_mode
rosrun rviz rviz  -d `rospack find hsrb_common_launch`/config/hsrb_display_full_hsrb.rviz
```

## Tasks
1. Nilp - Object Manipulation 
2. Nikolas - Object detection
3. Sahil - Task planning

