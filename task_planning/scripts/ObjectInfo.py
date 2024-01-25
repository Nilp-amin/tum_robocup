from __future__ import annotations

import rospy
import math
import tf

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped 

"""Encapsulates information about detected objects. 
"""
class ObjectInfo(object):
    UNIQUE_MIN_DIST = 2e-2
    Z_CENTROID_OFFSET = 0.1
    def __init__(self, marker: Marker):
        marker.pose.position.z -= ObjectInfo.Z_CENTROID_OFFSET 
        # convert the marker coordinates to the map frame
        if marker.header.frame_id != "map":
            centroid_stamped = PointStamped(header=marker.header, 
                                            point=marker.pose.position) 
            tf_listener = tf.TransformListener()
            tf_listener.waitForTransform("map",
                                         marker.header.frame_id,
                                         centroid_stamped.header.stamp,
                                         rospy.Duration(1.0))
            transformed_centroid = tf_listener.transformPoint("map", centroid_stamped)
            marker.pose.position = transformed_centroid.point
            marker.header.frame_id = "map"
        self._marker = marker

        # obtain dropoff point of object based on class
        if marker.text in ["Cup", "Bottle", "Pringles"]:
            drop_coords = rospy.get_param(f"/way_points/drop_{marker.text}") 
            drop_coords_stamped = PointStamped()
            drop_coords_stamped.header.frame_id = rospy.get_param(f"/way_points/frame",
                                                                default="map")
            drop_coords_stamped.header.stamp = rospy.Time.now()
            drop_coords_stamped.point = Point(drop_coords["x"],
                                            drop_coords["y"],
                                            drop_coords["z"])
            self._dropoff_point = drop_coords_stamped

        # possible locations in the map frame to go to pickup this object
        self._pickup_locations = []

    def get_position(self) -> PointStamped:
        """Gets the centroid of the object in the map frame.
        """
        point_stamped = PointStamped(header=self._marker.header,
                                     point=self._marker.pose.position)
        return point_stamped

    def get_class(self) -> str:
        """Gets the class of the object.
        """
        return self._marker.text
    
    def get_class_id(self) -> int:
        """Gets the class id of the object as seen in the
        point cloud.
        """
        return self._marker.id

    def get_dropoff_point(self) -> PointStamped:
        """Gets the location of the dropoff point
        for this object in the map frame. Dropoff point 
        is based on the class of the object.
        """
        return self._dropoff_point

    def get_pickup_location(self, index:int) -> PointStamped:
        """Gets the pickup location at give index.
        """
        return self._pickup_locations[index % len(self._pickup_locations)]

    def add_pickup_location(self, location:PointStamped) -> None:
        """Adds a new pickup location for this object.
        The pickup location should be in the map frame.
        """
        self._pickup_locations.append(location)

    def __eq__(self, other: ObjectInfo) -> bool:
        """Check if other is the same as this. Uses the
        eucledian distance between the objects centroids
        and compares it to some constant. If distance is below
        this constant then they are probably the same object.
        """
        position = self.get_position().point
        other_position = other.get_position().point
        x0 = position.x
        y0 = position.y
        z0 = position.z
        x1 = other_position.x 
        y1 = other_position.y
        z1 = other_position.z

        dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2 + (z1 - z0)**2)
        return dist < ObjectInfo.UNIQUE_MIN_DIST 