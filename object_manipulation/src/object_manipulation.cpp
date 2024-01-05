#include <object_manipulation/object_manipulation.h>

ObjectManipulation::ObjectManipulation(ros::NodeHandle& nh,
                                       const std::string& labeled_objects_topic,
                                       const std::string& camera_point_cloud_topic)
: nh_{nh},
  labeled_objects_cloud_topic_{labeled_objects_topic},
  camera_point_cloud_topic_{camera_point_cloud_topic},
  move_group_{"arm_torso"},
  visual_tools_{"base_footprint"} {}

bool ObjectManipulation::initalise()
{
    labeled_object_cloud_sub_.subscribe(nh_, labeled_objects_cloud_topic_, 10);
    camera_point_cloud_sub_.subscribe(nh_, camera_point_cloud_topic_, 10);
    sync_sub_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), labeled_object_cloud_sub_, camera_point_cloud_sub_));
    sync_sub_->registerCallback(boost::bind(&ObjectManipulation::cloudCallback, this, _1, _2));

    gpd_ros_grasps_sub_ = nh_.subscribe("/detect_grasps/clustered_grasps", 1, &ObjectManipulation::graspsCallback, this);

    gpd_ros_cloud_pub_ = nh_.advertise<gpd_ros::CloudSamples>("/cloud_stitched", 10);

    // set moveit configurations
    move_group_.setPlannerId("RRTConnectkConfigDefault");
    move_group_.setPlanningTime(2.0);
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

void ObjectManipulation::createPlanningScene(const std::string& label)
{
    // obtain the currently detected labels
    visualization_msgs::MarkerArrayConstPtr labels = 
        ros::topic::waitForMessage<visualization_msgs::MarkerArray>(
            "/text_markers", nh_, ros::Duration{2.0}
        );
    
    // find the pose of the target object centroid
    bool target_object_found{false};
    geometry_msgs::Pose target_object_pose;
    if (labels != nullptr)
    {
        for (size_t i{0}; i < labels->markers.size(); ++i)
        {
            if (labels->markers[i].text == label)
            {
                target_object_found = true;
                target_object_pose = labels->markers[i].pose;
                target_object_pose.position.z -= 0.1;
                target_object_pose.orientation.x = 0.0;
                target_object_pose.orientation.y = 0.0;
                target_object_pose.orientation.z = 0.0;
                target_object_pose.orientation.w = 1.0;
                break;
            }
        }
        if (!target_object_found)
        {
            ROS_ERROR_STREAM("no label: " << label << " found");
        }
    } else
    {
        ROS_ERROR("no labels detected");
    }

    // find the verticies of the plane to avoid collision with
    sensor_msgs::PointCloud2ConstPtr plane_verticies = 
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            "/table_vertices", nh_, ros::Duration{2.0}
        );


    if (target_object_found && plane_verticies != nullptr)
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
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = plane_verticies->header.frame_id;
        collision_object.id = "plane_object";

        // define the pose of the box in the planning scene representing the plane
        geometry_msgs::Pose plane_pose;
        plane_pose.position.x = (max_x + min_x) / 2.0;
        plane_pose.position.y = (max_y + min_y) / 2.0;
        // plane_pose.position.z = (max_z + min_z) / 2.0 - 0.02;
        plane_pose.position.z = (max_z + min_z) / 2.0;
        plane_pose.orientation.w = 1.0;

        // define the shape of the plane box object
        shape_msgs::SolidPrimitive plane_primitive;
        plane_primitive.type = plane_primitive.BOX;
        plane_primitive.dimensions.resize(3);
        plane_primitive.dimensions[plane_primitive.BOX_X] = std::abs(max_x - min_x);
        plane_primitive.dimensions[plane_primitive.BOX_Y] = std::abs(max_y - min_y);
        plane_primitive.dimensions[plane_primitive.BOX_Z] = 0.05;

        // define the shape of the target box object
        shape_msgs::SolidPrimitive target_primitive;
        target_primitive.type = target_primitive.BOX;
        target_primitive.dimensions.resize(3);
        target_primitive.dimensions[target_primitive.BOX_X] = 0.12;
        target_primitive.dimensions[target_primitive.BOX_Y] = 0.12;
        target_primitive.dimensions[target_primitive.BOX_Z] = 0.12;

        collision_object.primitives.push_back(plane_primitive);
        collision_object.primitive_poses.push_back(plane_pose);
        collision_object.primitives.push_back(target_primitive);
        collision_object.primitive_poses.push_back(target_object_pose);

        // add objects to the planning scene
        planning_interface_.applyCollisionObject(collision_object);
    } else
    {
        ROS_ERROR("no collision objects added to planning scene");
    }
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
        // std::cout << "Published cloud sample" << std::endl;
    }
}

void ObjectManipulation::graspsCallback(const gpd_ros::GraspConfigListConstPtr& msg)
{
    size_t idx{0};
    bool success{false};
    while (success == false && idx < msg->grasps.size())
    {
        ROS_INFO_STREAM("selected grasp configuration with score: " << msg->grasps[idx].score);
        gpd_ros::GraspConfig grasp{msg->grasps[idx]};

        // set up a pose goal
        geometry_msgs::Pose target_pose;
        target_pose.position.x = grasp.position.x;
        target_pose.position.y = grasp.position.y;
        // target_pose.position.z = grasp.position.z + 0.2;
        target_pose.position.z = grasp.position.z;

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
        Eigen::Vector3d shift_x_axis{-0.25, 0.0, 0.0};
        auto shifted_position = T_base_target * shift_x_axis;
        target_pose.position.x = shifted_position.x();
        target_pose.position.y = shifted_position.y();
        target_pose.position.z = shifted_position.z();

        move_group_.setPoseTarget(target_pose);
        visual_tools_.publishAxis(target_pose, rviz_visual_tools::LARGE);
        visual_tools_.trigger();

        // generate planning scene
        ////TODO: make it generalised
        createPlanningScene("cup");

        moveit::planning_interface::MoveGroupInterface::Plan move_plan;
        success = (move_group_.plan(move_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        ROS_INFO("plan (pose goal) %s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            move_group_.execute(move_plan);

            ////TODO: now move along an axis towards the object
            planning_interface_.removeCollisionObjects(planning_interface_.getKnownObjectNames());
            shift_x_axis[0] = 0.13;
            T_base_target = poseMsgToEigen(target_pose);
            shifted_position = T_base_target * shift_x_axis;
            target_pose.position.x = shifted_position.x();
            target_pose.position.y = shifted_position.y();
            target_pose.position.z = shifted_position.z();
            move_group_.setPoseTarget(target_pose);
            move_group_.move();
        }
        idx++;
    }

    if (success == false)
    {
        ROS_ERROR("No grasp pose was possible for this object.");
    }
    // planning_interface_.removeCollisionObjects(planning_interface_.getKnownObjectNames());

}