#ifndef OBJECT_MANIPULATION_H
#define OBJECT_MANIPULATION_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


#include <gpd_ros/CloudSamples.h>

#include <iostream>
#include <string>

class ObjectManipulation
{
public:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
public:
    ObjectManipulation(const std::string& labeled_objects_topic,
                       const std::string& camera_point_cloud_topic);

    bool initalise(ros::NodeHandle& nh);

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& labeled_msg,
                       const sensor_msgs::PointCloud2ConstPtr& camera_msg);

private:
    std::string labeled_objects_cloud_topic_;   // labeled point cloud topic name
    std::string camera_point_cloud_topic_;      // camera point cloud topic name

    // synchronised subscribers required for gpd
    message_filters::Subscriber<sensor_msgs::PointCloud2> labeled_object_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> camera_point_cloud_sub_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_sub_;

    ros::Publisher gpd_ros_cloud_pub_;          // publisher to gpd_ros

    tf::TransformListener tf_listener_;         // access to tf tree for ros transformations

};

#endif