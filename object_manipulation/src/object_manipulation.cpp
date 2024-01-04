#include <object_manipulation/object_manipulation.h>

ObjectManipulation::ObjectManipulation(const std::string& labeled_objects_topic,
                                       const std::string& camera_point_cloud_topic)
: labeled_objects_cloud_topic_{labeled_objects_topic},
  camera_point_cloud_topic_{camera_point_cloud_topic},
  move_group_{"arm_torso"},
  visual_tools_{"base_footprint"} {}

bool ObjectManipulation::initalise(ros::NodeHandle& nh)
{
    labeled_object_cloud_sub_.subscribe(nh, labeled_objects_cloud_topic_, 10);
    camera_point_cloud_sub_.subscribe(nh, camera_point_cloud_topic_, 10);
    sync_sub_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), labeled_object_cloud_sub_, camera_point_cloud_sub_));
    sync_sub_->registerCallback(boost::bind(&ObjectManipulation::cloudCallback, this, _1, _2));

    gpd_ros_grasps_sub_ = nh.subscribe("/detect_grasps/clustered_grasps", 1, &ObjectManipulation::graspsCallback, this);

    gpd_ros_cloud_pub_ = nh.advertise<gpd_ros::CloudSamples>("/cloud_stitched", 10);

    // set moveit configurations
    move_group_.setPlannerId("RRTConnectkConfigDefault");
    visual_tools_.loadRobotStatePub("/display_robot_state");

    return true;
}

Eigen::Affine3d ObjectManipulation::poseMsgToEigen(const geometry_msgs::Pose& pose_msg)
{
    Eigen::Affine3d transformation_matrix;

    // Translation vector
    Eigen::Vector3d translation(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);

    // Quaternion for rotation
    Eigen::Quaterniond rotation(
        pose_msg.orientation.w,
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z
    );

    // Set the transformation matrix
    transformation_matrix = Eigen::Translation3d(translation) * rotation;

    return transformation_matrix;
}

void ObjectManipulation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& labeled_cloud_msg,
                                       const sensor_msgs::PointCloud2ConstPtr& camera_cloud_msg)
{
    // obtain the camera position at the provided cloud_msg timestamp
    tf::StampedTransform T_base_camera;
    tf_listener_.lookupTransform(
        "base_footprint",
        "xtion_rgb_optical_frame",
        ros::Time(0),
        T_base_camera
    );

    // populate the merged cloud information
    gpd_ros::CloudSources gpd_cloud_msg; 
    gpd_cloud_msg.cloud = *camera_cloud_msg;
    gpd_cloud_msg.camera_source = std::vector<std_msgs::Int64>{
        camera_cloud_msg->width * camera_cloud_msg->height,
        std_msgs::Int64{} 
    };
    geometry_msgs::Point camera_position;
    camera_position.x = T_base_camera.getOrigin().x();
    camera_position.y = T_base_camera.getOrigin().y();
    camera_position.z = T_base_camera.getOrigin().z();
    gpd_cloud_msg.view_points = std::vector<geometry_msgs::Point>{camera_position};

    // populate the gpd cloud samples msg for publishing to gpd_ros
    gpd_ros::CloudSamples gpd_cloud_samples_msg;
    gpd_cloud_samples_msg.cloud_sources = gpd_cloud_msg;

    // create a vector of points for which to search for grasp poses
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*labeled_cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*labeled_cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*labeled_cloud_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<int> iter_label(*labeled_cloud_msg, "label");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_label)
    {
        ////TODO: for now we only try and detect a cup, which is labeled as 4 
        if (*iter_label == 4)
        {
            geometry_msgs::Point sample_point;
            sample_point.x = *iter_x;
            sample_point.y = *iter_y;
            sample_point.z = *iter_z;
            gpd_cloud_samples_msg.samples.push_back(sample_point);
        }
    }

    // publish to gpd_ros
    if (gpd_cloud_samples_msg.samples.size() > 0)
    {
        gpd_ros_cloud_pub_.publish(gpd_cloud_samples_msg);
        std::cout << "Published cloud sample" << std::endl;
    }
}

void ObjectManipulation::graspsCallback(const gpd_ros::GraspConfigListConstPtr& msg)
{
    bool success = false;
    // // find the highest score grasp configuration
    // size_t max_grasp_score_idx{0};
    // float max_grasp_score = -std::numeric_limits<float>::max();
    // for (size_t i{0}; i < msg->grasps.size(); ++i)
    // {
    //     if (msg->grasps[i].score.data > max_grasp_score)
    //     {
    //         max_grasp_score_idx = i;
    //         max_grasp_score = msg->grasps[i].score.data;
    //     }
    // }

    // obtain the best grasp pose - always idx=0 in msg
    size_t idx{0};
    while (success == false && idx < msg->grasps.size())
    {
        ROS_INFO_STREAM("selected grasp configuration with score: " << msg->grasps[idx].score);
        gpd_ros::GraspConfig grasp{msg->grasps[idx]};

        // set up a pose goal
        geometry_msgs::Pose target_pose;
        target_pose.position.x = grasp.position.x;
        target_pose.position.y = grasp.position.y;
        target_pose.position.z = grasp.position.z + 0.2;

        // convert vectors to rotation matrix
        tf2::Matrix3x3 rotation_matrix{
            grasp.approach.x, grasp.binormal.x, grasp.axis.x,
            grasp.approach.y, grasp.binormal.y, grasp.axis.y,
            grasp.approach.z, grasp.binormal.z, grasp.axis.z,
        };

        // convert the rotation matrix into a quaternion
        tf2::Quaternion quaternion;
        rotation_matrix.getRotation(quaternion);

        tf2::Vector3 axis{1.0, 0.0, 0.0};
        double angle = M_PI / 2.0;

        quaternion *= tf2::Quaternion(axis, -angle);

        target_pose.orientation.x = quaternion.x();
        target_pose.orientation.y = quaternion.y();
        target_pose.orientation.z = quaternion.z();
        target_pose.orientation.w = quaternion.w();

        // before transformation
        visual_tools_.publishAxis(target_pose, rviz_visual_tools::LARGE);

        // obtain the transformation matrix from the base to the end effector 
        Eigen::Affine3d T_base_target = poseMsgToEigen(target_pose);
        // shift the target pose along its own x-axis by 0.12m
        Eigen::Vector3d shift_x_axis{-0.3, 0.0, 0.0};
        auto shifted_position = T_base_target * shift_x_axis;
        target_pose.position.x = shifted_position.x();
        target_pose.position.y = shifted_position.y();
        target_pose.position.z = shifted_position.z();

        move_group_.setPoseTarget(target_pose);
        visual_tools_.publishAxis(target_pose, rviz_visual_tools::LARGE);
        visual_tools_.trigger();
        // auto result = move_group_.move();
        moveit::planning_interface::MoveGroupInterface::Plan move_plan;

        success = (move_group_.plan(move_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        ROS_INFO("plan (pose goal) %s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            move_group_.execute(move_plan);
        }
        idx++;
    }

    if (success == false)
    {
        ROS_ERROR("No grasp pose was possible for this object.");
    }

}