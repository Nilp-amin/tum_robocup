#!/usr/bin/env python3

import random

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

SEED = 0
PACKAGE_NAME = "tmc_wrs_gazebo_worlds"

def spawn_sdf_model(model_name, sdf_file_path, spawn_pose=Pose()):
    # load the sdf model
    sdf_content = None
    with open(sdf_file_path, "r") as f:
        sdf_content = f.read()

    try:
        # call the SpawnModel service
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        response = spawn_model(model_name, sdf_content, "", spawn_pose, "world")

        if response.success:
            rospy.loginfo(f'{model_name} spawned successfully in Gazebo!')
        else:
            rospy.logerr(f'Failed to spawn model "{model_name}" in Gazebo. Error message: {response.status_message}')
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node("spawn_sdf_objects_node")

    # obtain the root sdf package file path 
    rospack = rospkg.RosPack()
    package_path = None
    try:
        package_path = rospack.get_path(PACKAGE_NAME)
        rospy.loginfo(f'The path of package {PACKAGE_NAME}: {package_path}')
    except rospkg.ResourceNotFound:
        rospy.logerr(f"Package {PACKAGE_NAME} not found.")

    try:
        spawn_sdf_model("banana", f"{package_path}/models/ycb_011_banana/model-1_4.sdf", Pose(
            position=Point(1.0, 0.0, 0.0), 
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        )
        spawn_sdf_model("meat_can", f"{package_path}/models/ycb_010_potted_meat_can/model-1_4.sdf", Pose(
            position=Point(0.5, 0.0, 0.0), 
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        )
        spawn_sdf_model("apple", f"{package_path}/models/ycb_013_apple/model-1_4.sdf", Pose(
            position=Point(0.8, 0.3, 0.0), 
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        )
        spawn_sdf_model("orange", f"{package_path}/models/ycb_017_orange/model-1_4.sdf", Pose(
            position=Point(0.54, 0.2, 0.0), 
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        )
        spawn_sdf_model("cup_a", f"{package_path}/models/ycb_065-a_cups/model-1_4.sdf", Pose(
            position=Point(0.7, 0.4, 0.0), 
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        )
        spawn_sdf_model("cup_d", f"{package_path}/models/ycb_065-d_cups/model-1_4.sdf", Pose(
            position=Point(1.2, 0.5, 0.0), 
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        )
        spawn_sdf_model("cup_i", f"{package_path}/models/ycb_065-i_cups/model-1_4.sdf", Pose(
            position=Point(1.1, 0.2, 0.0), 
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        )
    except rospy.ROSInterruptException:
        pass