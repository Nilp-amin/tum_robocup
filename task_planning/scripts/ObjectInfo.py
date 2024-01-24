from __future__ import annotations

import rospy
import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped 

"""Encapsulates information about detected objects. 
"""
class ObjectInfo(object):
    UNIQUE_MIN_DIST = 2e-2
    Z_CENTROID_OFFSET = 0.1
    def __init__(self, marker: Marker):
        self._eq_min_dist

        marker.pose.position.z -= ObjectInfo.Z_CENTROID_OFFSET 
        self._marker = marker

        # obtain dropoff point of object based on class
        drop_coords = rospy.get_param(f"/way_points/drop_{marker.text}") 
        drop_coords_stamped = PointStamped()
        drop_coords_stamped.header.frame_id = rospy.get_param(f"/way_points/frame",
                                                              default="map")
        drop_coords_stamped.point = Point(drop_coords["x"],
                                          drop_coords["y"],
                                          drop_coords["z"])
        self._dropoff_point = drop_coords_stamped

    def get_position(self) -> PointStamped:
        """Gets the centroid of the object.
        """
        point_stamped = PointStamped()
        point_stamped.header.frame_id = self._marker.header.frame_id
        point_stamped.point = self._marker.pose.position

        return point_stamped

    def get_class(self) -> str:
        """Gets the class of the object.
        """
        return self._marker.text

    def get_dropoff_point(self) -> PointStamped:
        """Gets the location of the dropoff point
        for this object. Based on the class of the object.
        """
        return self._dropoff_point

    def __eq__(self, other: ObjectInfo) -> bool:
        """Check if other is the same as this. Uses the
        eucledian distance between the objects centroids
        and compares it to some constant. If distance is below
        this constant then they are probably the same object.
        """
        x0 = self.get_position().x
        y0 = self.get_position().y
        z0 = self.get_position().z
        x1 = other.get_position().x 
        y1 = other.get_position().y
        z1 = other.get_position().z

        dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2 + (z1 - z0)**2)
        return dist < ObjectInfo.UNIQUE_MIN_DIST 