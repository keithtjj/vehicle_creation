#!/usr/bin/env python3

import yaml
import os
import rospy
from std_msgs.msg import Int32

script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(script_path)
project_dir = os.path.dirname(os.path.dirname(script_dir))
param_file = os.path.join(project_dir, "mqtt_bridge/config/la_params.yaml")

with open(param_file) as file:
    config = yaml.safe_load(file)

vehicle_name = config['vehicle']
vehicle_split = vehicle_name.split('/')
for i in vehicle_split:
    if i.isnumeric():
        vehicle_num = i

def publish_priority(priority):
    # Initialize the ROS node
    rospy.init_node('priority_publisher', anonymous=True)

    # Create a publisher for the priority topic
    pub = rospy.Publisher('Priority', Int32, queue_size=10)

    # Set the loop rate to 10 Hz
    rate = rospy.Rate(10)

    try:
        # Publish priority messages until the node is shut down
        while not rospy.is_shutdown():
            # Publish the priority number
            pub.publish(priority)
            # Wait for the next iteration of the loop
            rate.sleep()
    except KeyboardInterrupt:
        quit()

if __name__ == '__main__':
    publish_priority(int(vehicle_num))
