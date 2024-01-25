#ifndef OBJECT_MANIPULATION_H
#define OBJECT_MANIPULATION_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <actionlib/client/simple_action_client.h>
#include <object_manipulation/Dropoff.h>
#include <object_manipulation/Pickup.h>

#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/GraspConfigList.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PickupGoal.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/GripperTranslation.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shapes.h>

#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <limits>

class ObjectManipulation
{
public:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
public:
    ObjectManipulation(ros::NodeHandle& nh,
                       const std::string& labeled_objects_topic,
                       const std::string& camera_point_cloud_topic,
                       const std::string& target_label_topic);

    bool initalise();

private:
    double deg2rad(const double degrees);

    Eigen::Affine3d poseMsgToEigen(const geometry_msgs::Pose& pose_msg);

    void createPlanningScene(const geometry_msgs::PointStamped& target_centroid);

    moveit_msgs::PickupGoal createPickupGoal(const std::string& group="arm_torso",
                                             const std::string& target="part",
                                             const geometry_msgs::PoseStamped& grasp_pose=geometry_msgs::PoseStamped(),
                                             const std::vector<moveit_msgs::Grasp>& possible_grasps={},
                                             const std::vector<std::string>& links_to_allow_contact={});

    std::vector<moveit_msgs::Grasp> createGrasps(const gpd_ros::GraspConfigListConstPtr& grasps_msg);

    bool pickupCallback(object_manipulation::Pickup::Request&  req,
                        object_manipulation::Pickup::Response& res);

    bool dropoffCallback(object_manipulation::Dropoff::Request&  req,
                         object_manipulation::Dropoff::Response& res);

private:
    ros::NodeHandle nh_;

    tf::TransformListener tf_listener_;                                         // access to tf tree for ros transformations

    moveit::planning_interface::MoveGroupInterface move_group_;                 // moveit move interface for whole body
    moveit::planning_interface::PlanningSceneInterface planning_interface_;     // moveit planning scene interface

    ros::ServiceServer pickup_service_;
    ros::ServiceServer dropoff_service_; 

    ros::ServiceClient octomap_client_;

    ros::Subscriber gpd_ros_grasps_sub_;                                        // subscriber to gpd_ros of optimal grasps

    ros::Publisher gpd_ros_cloud_pub_;                                          // publisher to gpd_ros

private:
    std::vector<std::string> gripper_joint_names_;
    std::vector<float> gripper_pre_grasp_positions_;
    std::vector<float> gripper_grasp_positions_;

    float time_pre_grasp_posture_;
    float time_grasp_posture_;
    float time_grasp_posture_final_;

    std::string grasp_postures_frame_id_;
    std::string grasp_pose_frame_id_;

    float grasp_desired_distance_;
    float grasp_min_distance_;

    float pre_grasp_direction_x_;
    float pre_grasp_direction_y_;
    float pre_grasp_direction_z_;

    float post_grasp_direction_x_;
    float post_grasp_direction_y_;
    float post_grasp_direction_z_;

    float max_contact_force_;
    std::vector<std::string> allowed_touch_objects_;

    std::vector<std::string> links_to_allow_contact_;
};

#endif