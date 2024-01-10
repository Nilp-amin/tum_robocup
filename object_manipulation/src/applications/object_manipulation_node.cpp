#include <object_manipulation/object_manipulation.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_manipulation");
    ros::NodeHandle nh;

    // start async spinner for moveit
    ros::AsyncSpinner spinner{1};

    std::string labeled_objects_cloud_topic{"/labeled_object_point_cloud"};
    std::string camera_cloud_topic{"/combined_point_cloud"};
    std::string target_label_topic{"/target_label"};

    ObjectManipulation manipulation{nh, labeled_objects_cloud_topic, camera_cloud_topic, target_label_topic};

    // init
    if (!manipulation.initalise())
    {
        ROS_ERROR("Error init ObjectManipulation");
        return -1;
    }

    // run
    ros::Rate rate{30};
    spinner.start();
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    spinner.stop();
    ros::shutdown();

    return 0;
}