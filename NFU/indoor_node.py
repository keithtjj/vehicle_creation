#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from laser_geometry import LaserProjection

def is_indoor(point_cloud):
    # Extract the X, Y, and Z coordinates of the points
    x = point_cloud[:, 0]
    y = point_cloud[:, 1]
    z = point_cloud[:, 2]

    # Compute the range of the points in each dimension
    x_range = np.max(x) - np.min(x)
    y_range = np.max(y) - np.min(y)
    z_range = np.max(z) - np.min(z)

    # Compute the standard deviation of the X, Y, and Z coordinates
    x_std = np.std(x)
    y_std = np.std(y)
    z_std = np.std(z)

    # Check if the range and standard deviation are consistent with an indoor environment
    if x_range < 5 and y_range < 5 and z_range < 3 and x_std < 1 and y_std < 1 and z_std < 1:
        return True
    else:
        return False
    
def callback(data):
    # Convert the point cloud message into a numpy array
    pointcloud_arr = np.array(list(pc2.read_points(data)))

    # Extract the X, Y, and Z coordinates of the points
    x = pointcloud_arr[:, 0]
    y = pointcloud_arr[:, 1]
    z = pointcloud_arr[:, 2]

    # Create a new point cloud object using the extracted coordinates
    point_cloud = np.column_stack((x, y, z))

    # Analyze the structure of the point cloud to determine if it represents an indoor environment
    indoor_bool = is_indoor(point_cloud)
    # TODO: Implement code to analyze the point cloud and determine if it represents an indoor environment

    # Print the result
    if indoor_bool:
        rospy.loginfo("The point cloud represents an indoor environment")
    else:
        rospy.loginfo("The point cloud does not represent an indoor environment")

def listener():
    # Initialize the ROS node and subscribe to the point cloud topic
    rospy.init_node('point_cloud_listener', anonymous=True)
    rospy.Subscriber('/registered_scan', PointCloud2, callback)

    # Spin until shutdown
    rospy.spin()

if __name__ == '__main__':
    listener()

