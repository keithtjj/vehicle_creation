#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

def publish_availability(is_available):
    # Initialize the node with a unique name
    rospy.init_node('availability_publisher', anonymous=True)
    # Create a publisher for the availability topic
    pub = rospy.Publisher('Availability', Bool, queue_size=10)

    # Set the loop rate to 10 Hz
    rate = rospy.Rate(10)

    # Publish availability messages until the node is shut down
    while not rospy.is_shutdown():
        # Publish the current availability status
        pub.publish(is_available)
        # Wait for the next iteration of the loop
        rate.sleep()

if __name__ == '__main__':
    # Publish availability as True
    publish_availability(True)
