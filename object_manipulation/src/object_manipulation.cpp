#include <object_manipulation/object_manipulation.h>

ObjectManipulation::ObjectManipulation(ros::NodeHandle& nh,
                                       const std::string& labeled_objects_topic,
                                       const std::string& camera_point_cloud_topic,
                                       const std::string& target_label_topic)
: nh_{nh},
  move_group_{"whole_body_light"},
  gripper_{"gripper"},
  arm_{"arm"} {}

bool ObjectManipulation::initalise()
{
    ROS_INFO("Loading rosparams.");
    if (!ros::param::get("/object_manipulation_node/gripper_joint_names", gripper_joint_names_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/gripper_pre_grasp_positions", gripper_pre_grasp_positions_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/gripper_grasp_positions", gripper_grasp_positions_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/time_pre_grasp_posture", time_pre_grasp_posture_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/time_grasp_posture", time_grasp_posture_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/time_grasp_posture_final", time_grasp_posture_final_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/grasp_postures_frame_id", grasp_postures_frame_id_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/grasp_pose_frame_id", grasp_pose_frame_id_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/grasp_desired_distance", grasp_desired_distance_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/grasp_min_distance", grasp_min_distance_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/pre_grasp_direction_x", pre_grasp_direction_x_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/pre_grasp_direction_y", pre_grasp_direction_y_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/pre_grasp_direction_z", pre_grasp_direction_z_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/post_grasp_direction_x", post_grasp_direction_x_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/post_grasp_direction_y", post_grasp_direction_y_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/post_grasp_direction_z", post_grasp_direction_z_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/max_contact_force", max_contact_force_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/allowed_touch_objects", allowed_touch_objects_)) { return false; }
    if (!ros::param::get("/object_manipulation_node/links_to_allow_contact", links_to_allow_contact_)) { return false; }

    ROS_INFO("Initalising rostopics and rosservices.");

    gpd_ros_cloud_pub_ = nh_.advertise<gpd_ros::CloudSamples>("/cloud_stitched", 1);

    dropoff_service_ = nh_.advertiseService("dropoff", &ObjectManipulation::dropoffCallback, this);
    pickup_service_ = nh_.advertiseService("pickup", &ObjectManipulation::pickupCallback, this);

    octomap_client_ = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");

    // set moveit configurations
    move_group_.setPlannerId("RRTstarkConfigDefault");
    move_group_.setPoseReferenceFrame(grasp_pose_frame_id_);

    gripper_.setGoalJointTolerance(0.05);

    ROS_INFO("Initalisation complete.");

    return true;
}

double ObjectManipulation::deg2rad(const double degrees)
{
    return degrees * M_PI / 180.0;
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

void ObjectManipulation::createPlanningScene(const geometry_msgs::PointStamped& target_centroid)
{
    ROS_INFO("Removing any previous collision objects.");
    moveit_msgs::AttachedCollisionObject att_coll_object;
    att_coll_object.object.id = "target";
    att_coll_object.object.operation = att_coll_object.object.REMOVE;
    planning_interface_.applyAttachedCollisionObject(att_coll_object);
    planning_interface_.removeCollisionObjects(
        planning_interface_.getKnownObjectNames()
    );
    std_srvs::Empty octomap_srv;
    octomap_client_.call(octomap_srv);
    ros::Duration(2.0).sleep();

    // create pose message for target object
    geometry_msgs::PoseStamped target_object_pose;
    target_object_pose.header = target_centroid.header;
    target_object_pose.pose.position.x = target_centroid.point.x; 
    target_object_pose.pose.position.y = target_centroid.point.y;
    target_object_pose.pose.position.z = target_centroid.point.z;
    target_object_pose.pose.orientation.w = 1.0;

    // find the verticies of the plane to avoid collision with
    //// TODO: can probably just hard code verticies to make it more robust
    sensor_msgs::PointCloud2ConstPtr plane_verticies = 
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            "/table_vertices", nh_, ros::Duration{2.0}
        );


    if (plane_verticies != nullptr)
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*plane_verticies, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*plane_verticies, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*plane_verticies, "z");

        // you will always only have two points in this point cloud
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        
        min_x = *iter_x;
        min_y = *iter_y;
        min_z = *iter_z;

        max_x = *(++iter_x);
        max_y = *(++iter_y);
        max_z = *(++iter_z);

        // Create a plane collision object
        moveit_msgs::CollisionObject plane_collision_object;
        plane_collision_object.header.frame_id = plane_verticies->header.frame_id;
        plane_collision_object.id = "plane";

        // define the pose of the box in the planning scene representing the plane
        geometry_msgs::Pose plane_pose;
        plane_pose.position.x = (max_x + min_x) / 2.0;
        plane_pose.position.y = (max_y + min_y) / 2.0;
        plane_pose.position.z = -0.025;
        plane_pose.orientation.w = 1.0;

        // define the shape of the plane box object
        shape_msgs::SolidPrimitive plane_primitive;
        plane_primitive.type = plane_primitive.BOX;
        plane_primitive.dimensions.resize(3);
        plane_primitive.dimensions[plane_primitive.BOX_X] = std::abs(max_x - min_x);
        plane_primitive.dimensions[plane_primitive.BOX_Y] = std::abs(max_y - min_y);
        plane_primitive.dimensions[plane_primitive.BOX_Z] = 0.05;

        plane_collision_object.primitives.push_back(plane_primitive);
        plane_collision_object.primitive_poses.push_back(plane_pose);

        // create a target collision object
        moveit_msgs::CollisionObject target_collision_object;
        target_collision_object.header.frame_id = target_object_pose.header.frame_id;
        target_collision_object.id = "target";

        // define the shape of the target box object
        shape_msgs::SolidPrimitive target_primitive;
        target_primitive.type = target_primitive.BOX;
        target_primitive.dimensions.resize(3);
        target_primitive.dimensions[target_primitive.BOX_X] = 0.005;
        target_primitive.dimensions[target_primitive.BOX_Y] = 0.005;
        target_primitive.dimensions[target_primitive.BOX_Z] = 0.005;

        target_collision_object.primitives.push_back(target_primitive);
        target_collision_object.primitive_poses.push_back(target_object_pose.pose);

        // add objects to the planning scene
        planning_interface_.applyCollisionObject(plane_collision_object);
        ROS_INFO("Added plane collision object.");
        planning_interface_.applyCollisionObject(target_collision_object);
        ROS_INFO("Added target collision object.");
    } else
    {
        ROS_ERROR("no collision objects added to planning scene");
    }
}

moveit_msgs::PickupGoal ObjectManipulation::createPickupGoal(const std::string& group,
                                                             const std::string& target,
                                                             const geometry_msgs::PoseStamped& grasp_pose,
                                                             const std::vector<moveit_msgs::Grasp>& possible_grasps,
                                                             const std::vector<std::string>& links_to_allow_contact)
{
    moveit_msgs::PickupGoal pug;
    pug.target_name = target;
    pug.group_name = group; 
    pug.possible_grasps.insert(
        pug.possible_grasps.begin(), 
        possible_grasps.begin(), 
        possible_grasps.end()
    );
    pug.allowed_planning_time = 35.0;
    pug.planning_options.planning_scene_diff.is_diff = true;
    pug.planning_options.planning_scene_diff.robot_state.is_diff = true;
    pug.planning_options.plan_only = false;
    pug.planning_options.replan = true;
    pug.planning_options.replan_attempts = 1;
    pug.attached_object_touch_links.insert(
        pug.attached_object_touch_links.begin(),
        links_to_allow_contact.begin(),
        links_to_allow_contact.end()
    );

    return pug;
}

std::vector<moveit_msgs::Grasp> ObjectManipulation::createGrasps(const gpd_ros::GraspConfigListConstPtr& grasps_msg)
{
    std::vector<moveit_msgs::Grasp> grasps;

    for (size_t idx{0}; idx < grasps_msg->grasps.size(); ++idx)
    {
        gpd_ros::GraspConfig grasp{grasps_msg->grasps[idx]};

        moveit_msgs::Grasp moveit_grasp;
        moveit_grasp.id = "grasp_" + std::to_string(idx);

        trajectory_msgs::JointTrajectory pre_grasp_posture;
        pre_grasp_posture.header.frame_id = grasp_postures_frame_id_;
        pre_grasp_posture.joint_names.insert(
            pre_grasp_posture.joint_names.begin(),
            gripper_joint_names_.begin(),
            gripper_joint_names_.end()
        );

        trajectory_msgs::JointTrajectoryPoint jt_point;
        jt_point.time_from_start = ros::Duration(time_pre_grasp_posture_);
        jt_point.positions.insert(
            jt_point.positions.begin(),
            gripper_pre_grasp_positions_.begin(),
            gripper_pre_grasp_positions_.end()
        );
        pre_grasp_posture.points.push_back(jt_point);

        trajectory_msgs::JointTrajectory grasp_posture;
        grasp_posture.header.frame_id = grasp_postures_frame_id_;
        grasp_posture.joint_names.insert(
            grasp_posture.joint_names.begin(),
            gripper_joint_names_.begin(),
            gripper_joint_names_.end()
        );
        trajectory_msgs::JointTrajectoryPoint jt_point2;
        jt_point2.time_from_start = ros::Duration(2.0);
        jt_point2.effort = {-0.05};
        jt_point2.positions.insert(
            jt_point2.positions.begin(),
            gripper_grasp_positions_.begin(),
            gripper_grasp_positions_.end()
        );
        grasp_posture.points.push_back(jt_point2);

        moveit_grasp.pre_grasp_posture = pre_grasp_posture;
        moveit_grasp.grasp_posture = grasp_posture;

        geometry_msgs::PoseStamped grasp_pose;
        grasp_pose.header.frame_id = grasp_pose_frame_id_;
        grasp_pose.pose.position.x = grasp.position.x;
        grasp_pose.pose.position.y = grasp.position.y;
        grasp_pose.pose.position.z = grasp.position.z;

        // convert vectors to rotation matrix
        tf2::Matrix3x3 rotation_matrix{
            grasp.approach.x, grasp.binormal.x, grasp.axis.x,
            grasp.approach.y, grasp.binormal.y, grasp.axis.y,
            grasp.approach.z, grasp.binormal.z, grasp.axis.z,
        };

        // fix the rotation of the gripper from gpd_ros to hsrb convention
        tf2::Quaternion quaternion;
        rotation_matrix.getRotation(quaternion);
        // rotate about x-axis by 180 deg
        tf2::Quaternion x_quaternion;
        x_quaternion.setRPY(M_PI, 0.0, 0.0);
        quaternion *= x_quaternion;
        // rotate about y-axis by 90 deg
        tf2::Quaternion y_quaternion;
        y_quaternion.setRPY(0.0, M_PI_2, 0.0);
        quaternion *= y_quaternion;

        grasp_pose.pose.orientation.x = quaternion.x();
        grasp_pose.pose.orientation.y = quaternion.y();
        grasp_pose.pose.orientation.z = quaternion.z();
        grasp_pose.pose.orientation.w = quaternion.w();

        moveit_grasp.grasp_pose = grasp_pose;
        moveit_grasp.grasp_quality = grasp.score.data;

        moveit_grasp.pre_grasp_approach.direction.header.frame_id = grasp_postures_frame_id_;
        moveit_grasp.pre_grasp_approach.direction.vector.x = pre_grasp_direction_x_;
        moveit_grasp.pre_grasp_approach.direction.vector.y = pre_grasp_direction_y_;
        moveit_grasp.pre_grasp_approach.direction.vector.z = pre_grasp_direction_z_;
        moveit_grasp.pre_grasp_approach.desired_distance = grasp_desired_distance_;
        moveit_grasp.pre_grasp_approach.min_distance = grasp_min_distance_;

        moveit_grasp.post_grasp_retreat.direction.header.frame_id = grasp_postures_frame_id_;
        moveit_grasp.post_grasp_retreat.direction.vector.x = post_grasp_direction_x_;
        moveit_grasp.post_grasp_retreat.direction.vector.y = post_grasp_direction_y_;
        moveit_grasp.post_grasp_retreat.direction.vector.z = post_grasp_direction_z_;
        moveit_grasp.post_grasp_retreat.desired_distance = grasp_desired_distance_;
        moveit_grasp.post_grasp_retreat.min_distance = grasp_min_distance_;

        moveit_grasp.max_contact_force = max_contact_force_;
        moveit_grasp.allowed_touch_objects = allowed_touch_objects_;

        grasps.push_back(moveit_grasp);
        ROS_INFO_STREAM("inserted grasp configuration with score: " 
            << grasp.score
        );
    }
    
    return grasps;
}

bool ObjectManipulation::pickupCallback(object_manipulation::Pickup::Request&  req,
                                        object_manipulation::Pickup::Response& res)
{
    ROS_INFO_STREAM("Pickup service called for " << req.object_class << " with id " << req.object_id);
    res.succeeded = false;

    // obtain the camera position at the provided cloud_msg timestamp
    tf::StampedTransform T_base_camera;
    tf_listener_.lookupTransform(
        "base_footprint",
        "head_rgbd_sensor_rgb_frame",
        req.object_cloud.header.stamp,
        T_base_camera
    );

    // populate the merged cloud information
    gpd_ros::CloudSources gpd_cloud_msg; 
    gpd_cloud_msg.cloud = req.environment_cloud;
    gpd_cloud_msg.camera_source = std::vector<std_msgs::Int64>{
        req.environment_cloud.width * req.environment_cloud.height,
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
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(req.object_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(req.object_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(req.object_cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<int> iter_label(req.object_cloud, "label");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_label)
    {
        if (*iter_label == req.object_id)
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
        ROS_INFO("Publishing point cloud to gpd_ros.");
        gpd_ros_cloud_pub_.publish(gpd_cloud_samples_msg);
    }

    gpd_ros::GraspConfigListConstPtr grasp_config_list_msg =
        ros::topic::waitForMessage<gpd_ros::GraspConfigList>("/detect_grasps/clustered_grasps",
                                                             nh_,
                                                             ros::Duration(10.0));
    if (grasp_config_list_msg != nullptr)
    {
        ROS_INFO("Obtained possible grasp pose candidates from gpd_ros.");
        move_group_.setStartStateToCurrentState();
        createPlanningScene(req.object_centroid);
        std::vector<moveit_msgs::Grasp> possible_grasps = createGrasps(grasp_config_list_msg);
        moveit_msgs::PickupGoal goal = createPickupGoal(
            "whole_body_light",
            "target",
            geometry_msgs::PoseStamped{},
            possible_grasps,
            links_to_allow_contact_
        );
        ROS_INFO("Sending goal.");
        auto result = move_group_.pick(goal);
        ROS_INFO("Waiting for result.");
        ROS_INFO("Pick result: %s", result == moveit::core::MoveItErrorCode::SUCCESS ? "SUCCESS" : "FAILED");
        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            // open and close gripper to release any failed grasped objects
            gripper_.setJointValueTarget("hand_motor_joint", 1.2);
            gripper_.setJointValueTarget("hand_motor_joint", 0.0);
        } else
        {
            end_effector_pose_ = arm_.getCurrentPose();
        }

        res.succeeded = (result == moveit::core::MoveItErrorCode::SUCCESS);
    }

    return true;
}

bool ObjectManipulation::dropoffCallback(object_manipulation::Dropoff::Request&  req,
                                         object_manipulation::Dropoff::Response& res)
{
    // set pose to how the object was pick up
    arm_.setPoseTarget(end_effector_pose_);
    moveit::planning_interface::MoveGroupInterface::Plan drop_plan;

    bool success = (arm_.plan(drop_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Drop plan successful. Moveing to the target pose.");
        arm_.move();
    } else
    {
        ROS_ERROR("Drop plan failed. Unable to move to the target pose.");
    }

    res.succeeded = success;

    return true;
}