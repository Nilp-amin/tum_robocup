#include <object_labeling/object_labeling.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_labeling"); // TODO: change name???
  ros::NodeHandle nh;

  //#>>>>TODO: Set the correct topic names and frames
  std::string objects_cloud_topic{"/objects_point_cloud"}; // = "?"
  std::string camera_info_topic{"/hsrb/head_rgbd_sensor/depth_registered/camera_info"}; // = "?"
  std::string camera_frame{"head_rgbd_sensor_rgb_frame"}; // = "?" // optical frame??

  ObjectLabeling labeling(
    objects_cloud_topic,
    camera_info_topic,
    camera_frame);

  // Init
  if(!labeling.initalize(nh))
  {
    return -1;
  }

  // Run
  ros::Rate rate(30);
  ros::Duration(2.0).sleep();
  while(ros::ok())
  {
    labeling.update(ros::Time(0));
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
