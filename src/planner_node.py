#!/usr/bin/env python3

import yaml
import os
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped

import creation_node

import numpy as np
import random
import robot

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

tare_wp = PointStamped()

def store_tare_callback(data):
    #rospy.loginfo("Received point at time %d", data.header.stamp.to_sec())
    global tare_wp
    tare_wp = data
    #pub.publish(tare_wp)


if __name__ == '__main__':
    rospy.init_node('planner_node')
    rospy.Subscriber("/tare_way_point", PointStamped, store_tare_callback)
    global pub 
    pub = rospy.Publisher('/way_point', PointStamped, queue_size=10)
    rospy.spin()