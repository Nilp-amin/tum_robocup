#ifndef OBJECT_MANIPULATION_H
#define OBJECT_MANIPULATION_H

#include <ros/ros.h>
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
                       const std::string& camera_point_cloud_topic);

    bool initalise();

private:
    double deg2rad(const double degrees);

    Eigen::Affine3d poseMsgToEigen(const geometry_msgs::Pose& pose_msg);

    void createPlanningScene(const std::string& label);

    moveit_msgs::PickupGoal createPickupGoal(const std::string& group="arm_torso",
                                             const std::string& target="part",
                                             const geometry_msgs::PoseStamped& grasp_pose=geometry_msgs::PoseStamped(),
                                             const std::vector<moveit_msgs::Grasp>& possible_grasps={},
                                             const std::vector<std::string>& links_to_allow_contact={});

    std::vector<moveit_msgs::Grasp> createGrasps(const gpd_ros::GraspConfigListConstPtr& grasps_msg);
    
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& labeled_msg,
                       const sensor_msgs::PointCloud2ConstPtr& camera_msg);

    void graspsCallback(const gpd_ros::GraspConfigListConstPtr& msg);

private:
    ros::NodeHandle nh_;

    std::string labeled_objects_cloud_topic_;                                   // labeled point cloud topic name
    std::string camera_point_cloud_topic_;                                      // camera point cloud topic name

    tf::TransformListener tf_listener_;                                         // access to tf tree for ros transformations

    moveit::planning_interface::MoveGroupInterface move_group_;                 // moveit move interface
    moveit::planning_interface::PlanningSceneInterface planning_interface_;     // moveit planning scene interface
    moveit_visual_tools::MoveItVisualTools visual_tools_;

    actionlib::SimpleActionClient<moveit_msgs::PickupAction> pickup_ac_;

    // synchronised subscribers required for gpd
    message_filters::Subscriber<sensor_msgs::PointCloud2> labeled_object_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> camera_point_cloud_sub_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_sub_;

    ros::Subscriber gpd_ros_grasps_sub_;                                        // subscriber to gpd_ros of optimal grasps

    ros::Publisher gpd_ros_cloud_pub_;                                          // publisher to gpd_ros
};

#endif