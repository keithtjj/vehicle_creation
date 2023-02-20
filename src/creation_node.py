#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32MultiArray, Int32, Bool
from nav_msgs.msg import Odometry, Path
import time 
import math
import rospy
from geometry_msgs.msg import Twist


vehicle_name_list = []
vehicle_list = []


class Timer:
    def __init__(self):
        self.start_time = None
        self.elapsed_time = None

    def start(self):
        self.start_time = time.time()

    def stop(self):
        if self.start_time is not None:
            self.elapsed_time = time.time() - self.start_time
            self.start_time = None
        else:
            raise ValueError("Timer not started")

    def reset(self):
        self.start_time = None
        self.elapsed_time = None

class Vehicle:
    def __init__(self, topic_name, vehicle_index, name):
        # Implement vehicle initialization logic here
        # Initalise all object values
        self.topic_name = topic_name
        self.number = vehicle_index
        self.name = name
        self.availability = False

        self._last_available = None
        
        self.obstacle_threshold = 1.0 # Distance threshold in meters

        self.topics = self.get_vehicle_topics()

        #Subscribe to availability
        self.subavail(self.topics)

        #Subscribe to topics
        self.subtopics(self.topics)

        #Publish topics
        self.pubtopics()
        
        #subscribe to local path
        self.local_path_sub = rospy.Subscriber("/sensor_coverage_planner/local_path", Path, self.path_callback)


    def get_vehicle_topics(self):
        # Get a list of all published topics
        topics = rospy.get_published_topics()

        # Filter the list of topics to only include those with vehicle number included
        vehicle_topics = [[topic_name, _] for topic_name, _ in topics if (topic_name.find(self.number)!=(-1))]

        return vehicle_topics
    
    #subscribe to availability
    def subavail(self, topics):
        for topic,msg_type in topics:
            if (topic.endswith("availability")):
                self.avail_sub = rospy.Subscriber(topic, Bool, self.avail_callback)
                print(self.name, "Subscribed to" , topic)
    
    #subscribe to topics
    def subtopics(self, topics):
        for topic,msg_type in topics:
            if (topic.endswith("Odometry")):
                self.odom_sub = rospy.Subscriber(topic, Odometry, self.odometry_callback)
                print(self.name, "subscribed to Odometry")
            if (topic.endswith("ExploringSubspacesIndices")):
                self.exploring_sub = rospy.Subscriber(topic, Int32MultiArray, self.exploring_subspace_callback)
                print(self.name, "subscribed to indices")
            if (topic.endswith("CoveredSubspacesIndices")):
                self.covered_sub = rospy.Subscriber(topic, Int32MultiArray, self.covered_subspace_callback)
                print(self.name, "subscribed to indices")
            if (topic.endswith("Priority")):
                self.priority_sub = rospy.Subscriber(topic, Int32, self.priority_callback)
                print(self.name, "subscribed to priority")

    #Publish topics
    def pubtopics(self):
        self.publisher = rospy.Publisher(f"{self.name}/Odometry", Odometry, queue_size=10)

    def update_vehicle(self):
        now = rospy.get_time()
        if self.availability == True:
            if now - self._last_available >= 30:
                self.availability = False
                rospy.loginfo("%s availability is now: %s", self.name, self.availability)


    def avail_callback(self, message):
        self.availability = message.data
        self._last_available = rospy.get_time()
        # print(self.name, "is available:", self.availability)
    
    def odometry_callback(self, odom_msg):
        # Extract the position and orientation data from the Odometry message
        self.position = odom_msg.pose.pose.position
        self.orientation = odom_msg.pose.pose.orientation

        # Check if vehicle odometry interferes with local path
        for i in range(len(self.positions)):
            if self.isInsideCircularBoundary(self.odometry.x, self.odometry.y, self.odometry.z, self.obstacle_threshold, self.positions[i].position.x, self.positions[i].position_.y, self.positions[i].position_.z):
                self.redflag = 1
                print("working")

        rospy.loginfo("Received odometry message: position = %s, orientation = %s", self.position, self.orientation)

    def exploring_subspace_callback(self, array_msg):
        self.exploring_cells_indices = array_msg.data

    def covered_subspace_callback(self, array_msg):
        self.covered_cells_indices = array_msg.data
    
    def priority_callback(self, int_msg):
        self.priority = int_msg.data

    def path_callback(self, path_msg):
        self.positions = []
        for pose_stamped in path_msg.poses:
            position = pose_stamped.pose.position
            self.positions.append([position])

    def isInsideCircularBoundary(centerX, centerY, centerZ, radius, pointX, pointY, pointZ):
        distance = math.sqrt((pointX - centerX) ** 2 + (pointY - centerY) ** 2 + (pointZ - centerZ) ** 2)
        return distance <= radius

class CollisionAvoidance:
    def __init__(self, vehicle_object):
        self.vehicle = vehicle_object
        self.odometry = vehicle_object.position
        self.name = vehicle_object.name
        self.local_path_sub = rospy.Subscriber("/sensor_coverage_planner/local_path", Path, self.path_callback)

        self.obstacle_avoidance_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.other_vehicles_sub = rospy.Subscriber('/other_vehicle/odom', Odometry, self.other_vehicles_callback)

    def path_callback(self, path_msg):
        self.positions = []
        for pose_stamped in path_msg.poses:
            position = pose_stamped.pose.position
            self.positions.append([position])
        for i in range(len(self.positions)):
            if self.isInsideCircularBoundary(self.odometry.x, self.odometry.y, self.odometry.z, self.obstacle_threshold, self.positions[i].position.x, self.positions[i].position_.y, self.positions[i].position_.z):
                self.redflag = 1
                print("working")


    def other_vehicles_callback(self, odom):
        # Check distance to other vehicle
        distance = odom.pose.pose.position.x # Assuming other vehicle's position in x-axis
        if distance < self.obstacle_threshold:
            # Obstacle detected! Avoid it.
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 1.0 # Turn to the right
            self.obstacle_avoidance_pub.publish(twist)

    def isInsideCircularBoundary(centerX, centerY, centerZ, radius, pointX, pointY, pointZ):
        distance = math.sqrt((pointX - centerX) ** 2 + (pointY - centerY) ** 2 + (pointZ - centerZ) ** 2)
        return distance <= radius


def generate_vehicle(topic, vehicle_num, name):
    if name not in vehicle_name_list:
        # Create a new instance of the Vehicle class with the specified make, model, and year
        vehicle = Vehicle(topic, vehicle_num, name)

        #Add Vehicle Created to list of created vehicles
        vehicle_name_list.append(vehicle.name)
        vehicle_list.append(vehicle)  

        # Return the new vehicle object
        return vehicle

def availtopics():
    #This function adds a new key-value pair for each new availability topic in ROS using rospy.

    # Get a list of all published topics
    topics = rospy.get_published_topics()

    # Filter the list of topics to only include those with the header "/availability"
    availability_topics = [topic_name for topic_name, _ in topics if (topic_name.find('availability')!=(-1))]

    #Dictionary for availability topics
    avail_dict = {}

    #Filter Topic name for numbers and name vehicle according to number
    for topic_name in availability_topics:
        split = topic_name.split('/')
        for elem in split:
            if elem.isnumeric():
                key_name = f"vehicle_{elem}" # Generate a key name like "vehicle_1", "vehicle_2", etc.
                if key_name not in avail_dict:
                    avail_dict[key_name] = topic_name
                    #check if vehicle has generated object, if none create object
                    if key_name not in vehicle_name_list:
                        key_name = generate_vehicle(topic_name, elem, key_name)

def updateVehicleStatus(vehicles):
    for vehicle in vehicles:
        vehicle.update_vehicle()
        if vehicle.redflag == 1:
            print(vehicle.name + "is invading personal space")
        print("working")

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('vehicle_manager')


r = rospy.Rate(1) # 10hz
while not rospy.is_shutdown():

    availtopics()
    updateVehicleStatus(vehicle_list)





