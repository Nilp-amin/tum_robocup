#include <object_manipulation/object_manipulation.h>

ObjectManipulation::ObjectManipulation(const std::string& labeled_objects_topic,
                                       const std::string& camera_point_cloud_topic)
: labeled_objects_cloud_topic_{labeled_objects_topic},
  camera_point_cloud_topic_{camera_point_cloud_topic} {}

bool ObjectManipulation::initalise(ros::NodeHandle& nh)
{
    // labeled_object_cloud_sub_ = nh.subscribe(labeled_objects_cloud_topic_, 10, &ObjectManipulation::cloudCallback, this);
    labeled_object_cloud_sub_.subscribe(nh, labeled_objects_cloud_topic_, 10);
    camera_point_cloud_sub_.subscribe(nh, camera_point_cloud_topic_, 10);
    sync_sub_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), labeled_object_cloud_sub_, camera_point_cloud_sub_));
    // sync_sub_.connectInput(labeled_object_cloud_sub_, camera_point_cloud_sub_);
    sync_sub_->registerCallback(boost::bind(&ObjectManipulation::cloudCallback, this, _1, _2));


    return true;
}

void ObjectManipulation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& labeled_msg, const sensor_msgs::PointCloud2ConstPtr& camera_msg)
{
    // sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    // int num_points{0};

    // for (; iter_x != iter_x.end(); ++iter_x) {
    //     num_points++;
    // }

    std::cout << "number of labeled points: " << labeled_msg->width * labeled_msg->height <<  std::endl;
    std::cout << "number of camera points: " << camera_msg->width * camera_msg->height <<  std::endl;

    // gpd_ros::detect_grasps srv;
    // srv.request.cloud_indexed 

}