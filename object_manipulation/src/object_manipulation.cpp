#include <object_manipulation/object_manipulation.h>

ObjectManipulation::ObjectManipulation(const std::string& labeled_objects_topic,
                                       const std::string& camera_point_cloud_topic)
: labeled_objects_cloud_topic_{labeled_objects_topic},
  camera_point_cloud_topic_{camera_point_cloud_topic} {}

bool ObjectManipulation::initalise(ros::NodeHandle& nh)
{
    labeled_object_cloud_sub_.subscribe(nh, labeled_objects_cloud_topic_, 10);
    camera_point_cloud_sub_.subscribe(nh, camera_point_cloud_topic_, 10);
    sync_sub_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), labeled_object_cloud_sub_, camera_point_cloud_sub_));
    sync_sub_->registerCallback(boost::bind(&ObjectManipulation::cloudCallback, this, _1, _2));

    gpd_ros_cloud_pub_ = nh.advertise<gpd_ros::CloudSamples>("/cloud_stitched", 10);

    return true;
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
    // std::vector<geometry_msgs::Point> samples;
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
    // gpd_cloud_samples_msg.samples = samples;

    // publish to gpd_ros
    if (gpd_cloud_samples_msg.samples.size() > 0)
    {
        gpd_ros_cloud_pub_.publish(gpd_cloud_samples_msg);
        std::cout << "Published cloud sample" << std::endl;
    }

    // gpd_ros_cloud_pub_.publish(gpd_cloud_samples_msg);
    // std::cout << "Published cloud sample" << std::endl;
}