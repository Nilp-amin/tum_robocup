# This message holds a point cloud of the environment and
# a labeled point cloud of the graspable objects in the environment.
# Information about the label and type of object to be grasped is 
# also provided.

# The point cloud of the environment.
sensor_msgs/PointCloud2 environment_cloud

# The labeled point cloud representing objects which can be grasped.
sensor_msgs/PointCloud2 object_cloud

# The target object class
std_msgs/String object_class

# The target object id in object_cloud 
std_msgs/Int64 object_id