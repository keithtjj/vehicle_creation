#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32MultiArray, Int32, Bool, Header
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped, PointStamped, Point
import yaml
import os
import numpy as np
from sensor_msgs.msg import PointCloud2
# from octomap_msgs.msg import Octomap
# from octomap_msgs import binary_octomap
from visibility_graph_msg.msg import Graph
from visualization_msgs.msg import Marker
from tare_msgs.msg import SubspaceArray, NodeAndEdge

import numpy as np

script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(script_path)
project_dir = os.path.dirname(os.path.dirname(script_dir))
param_file = os.path.join(project_dir, "mqtt_bridge/config/la_params.yaml")

vehicle_name_list = []
vehicle_list = []
vehicle_index_list = []
exploring_cell_indices = [0]
covered_cell_indices = [0]

generators = []
generators_name = []

corners = np.array([[-100.0, -100.0, 100.0, 100.0], 
					[-100.0, 100.0, 100.0, -100.0]])
safe_rad = 0.2

with open(param_file) as file:
    config = yaml.safe_load(file)

vehicle_name = config['vehicle']
vehicle_split = vehicle_name.split('/')
for i in vehicle_split:
    if i.isnumeric():
        vehicle_num = i

tare_list = []
tare_name_list = []
tare_wp = PointStamped()
current_pose = PoseStamped()
best_vg = Graph()

poi_list = []

class point:
    def __init__(self,x=0,y=0,z=0):
        self.x = x
        self.y = y
        self.z = z
    def __repr__(self):
        return f'({self.x}, {self.y}, {self.z})'
    def getxy(self):
        arr = [self.x,self.y]
        return arr

class Vehicle:
    def __init__(self, topic_name, vehicle_index, name):
        # Implement vehicle initialization logic here
        # Initalise all object values
        self.position = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.topic_name = topic_name
        self.number = vehicle_index
        self.name = name
        self.availability = False
        self._last_available = None
        self.obstacle_threshold = 1.0 # Distance threshold in meters
        self.topics = self.get_vehicle_topics()
        self.local_path = []
        self.redflag = 0
        self.exploring_indices = []
        self.covered_indices = []
        self.pos = point()
        self.planner = None

        self.wp = PointStamped()
        self.point = point()

        #Subscribe to availability
        self.subavail(self.topics)

        #Subscribe to topics
        #self.subtopics(self.topics)
        self.subscribs = []
        self.sub_topics(self.topic_list)

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

    def exploring_subspace_callback(self, array_msg):
        self.exploring_indices = array_msg.data
        # print(self.exploring_cells_indices)

    def covered_subspace_callback(self, array_msg):
        self.covered_indices = array_msg.data
        # print(self.covered_cells_indices)
    
    def priority_callback(self, int_msg):
        self.priority = int_msg.data

    def path_callback(self, path_msg):
        for pose_stamped in path_msg.poses:
            position_x = pose_stamped.pose.position.x
            position_y = pose_stamped.pose.position.y
            position_z = pose_stamped.pose.position.z
            position = dict(position_x = pose_stamped.pose.position.x,position_y = pose_stamped.pose.position.y,position_z = pose_stamped.pose.position.z)
            self.local_path.append(position)

    def isInsideCircularBoundary(self, centerX, centerY, centerZ, radius, pointX, pointY, pointZ):
        distance = math.sqrt((pointX - centerX) ** 2 + (pointY - centerY) ** 2 + (pointZ - centerZ) ** 2)
        return distance <= radius

    def multi_waypoint_callback(self,waypoint):
        #now = rospy.Time.now()
        #if (waypoint.header.stamp.to_sec() > (now.to_sec()-5)):
        self.wp = waypoint
        self.point.x = waypoint.point.x
        self.point.y = waypoint.point.y
        self.point.z = waypoint.point.z

    def poi_callback(self, poi):
        global poi_list
        r=2
        if poi.header.frame_id == 'test':
            return
        for prev_poi in poi_list:
            if poi.header.frame_id != prev_poi.header.frame_id:
                continue
            dx = poi.point.x - prev_poi.point.x
            dy = poi.point.y - prev_poi.point.y
            dxy = dx**2 + dy **2
            if dxy < r**2:
                return 
        poi_list.append(poi)
        pub_poi.publish(poi)
    
    def vg_callback(self, vg):
        global best_vg
        if vg.size > best_vg.size:
            best_vg = vg
            pub_vg.publish(vg)
    
    def del_model_callback(self, name):
        pub_kill.publish(name)

    def pose_callback(self, msg):
        self.position = msg.pose.position
        self.pos.x = msg.pose.position.x
        self.pos.y = msg.pose.position.y
        self.pos.z = msg.pose.position.z
        self.orientation = msg.pose.orientation

    def keypose_callback(self, msg):
        if int(self.number) != int(vehicle_num):
            pub_keypose.publish(msg)

    topic_list = [["pose_stamp", PoseStamped, pose_callback], 
                  ["Exploring_subspaces", SubspaceArray, exploring_subspace_callback], 
                  ["Covered_subspaces", SubspaceArray, covered_subspace_callback], 
                  ["keypose_node", NodeAndEdge, keypose_callback],
                  ["Priority", Int32, priority_callback],
                  ["planner_waypoint", PointStamped, multi_waypoint_callback], 
                  ["poi",PointStamped, poi_callback], 
                  ["vgraph", Graph, vg_callback], 
                  ["del_model", String, del_model_callback]]
    
    def sub_topics(self, topic_list):
        if int(self.number) == 0:
            print('veh creator started')
            return
        for topic in topic_list:
            name = 'vehicle/%s/%s' % (self.number, topic[0])
            if topic[0] == 'Odometry':
                subby = rospy.Subscriber(name, topic[1], self.super_callback, callback_args=topic, queue_size=1)
            else:
                subby = rospy.Subscriber(name, topic[1], self.super_callback, callback_args=topic, queue_size=5)
            print('subbed to', name)
            self.subscribs.append(subby)
        print('subbed to %s topics' % self.name)

    def super_callback(self, msg, toppy):
        for topic in self.topic_list:
            if toppy == topic:
                topic[2](self, msg)

vehicle_veh = Vehicle('topic', '0', 'name')
def generate_vehicle(topic, vehicle_numb, name):
    if name not in vehicle_name_list:
        # Create a new instance of the Vehicle class with the specified topic, number, and name
        vehicle = Vehicle(topic, vehicle_numb, name)
        #Add Vehicle Created to list of created vehicles
        vehicle_name_list.append(vehicle.name)
        vehicle_list.append(vehicle)  
        vehicle_index_list.append(int(vehicle.number))
        if vehicle_numb == vehicle_num:
            global vehicle_veh
            vehicle_veh = vehicle
            print('i am %s' % vehicle_veh.name)
        # Return the new vehicle object
        return vehicle

def availtopics():
    #This function adds a new key-value pair for each new availability topic in ROS using rospy.

    # Get a list of all published topics
    topics = rospy.get_published_topics()

    # Filter the list of topics to only include those with the header "/Availability"
    availability_topics = [topic_name for topic_name, _ in topics if (topic_name.find('Availability')!=(-1))]

    #Dictionary for availability topics
    avail_dict = {}

    #Filter Topic name for numbers and name vehicle according to number
    for topic_name in availability_topics:
        split = topic_name.split('/')
        for elem in split:
            if (elem.isnumeric()):
                key_name = f"vehicle_{elem}" # Generate a key name like "vehicle_1", "vehicle_2", etc.
                if key_name not in avail_dict:
                    avail_dict[key_name] = topic_name
                    #check if vehicle has generated object, if none create object
                    if key_name not in vehicle_name_list:
                        key_name = generate_vehicle(topic_name, elem, key_name)
    
def updateVehicleStatus(vehicles):
    for vehicle in vehicles:
        # print(vehicle.name)
        vehicle.update_vehicle()
        if vehicle.redflag == 1:
            print(vehicle.name + " is invading personal space")

def pub_exploring_cell_indices(vehicles):
    exploring_array = SubspaceArray()
    exploring_cell_indices = []
    for vehicle in vehicles:
        if (vehicle.number != vehicle_num):
            for i in vehicle.exploring_indices:
                exploring_cell_indices.append(i)
    exploring_array.data = exploring_cell_indices
    exploring_array.size = len(exploring_cell_indices)
    exploring_indices_publisher.publish(exploring_array)

def pub_covered_cell_indices(vehicles):
    cover_array = SubspaceArray()
    covered_cell_indices = []
    for vehicle in vehicles:
        if (vehicle.number != vehicle_num):
            for i in vehicle.covered_indices:
                covered_cell_indices.append(i)
    cover_array.data = covered_cell_indices
    cover_array.size = len(covered_cell_indices)
    covered_indices_publisher.publish(cover_array)

def odom_cb(msg):
    global current_pose
    current_pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
    pub_pose.publish(current_pose)

def twp_cb(msg):
    global tare_wp
    tare_wp = msg

# Check if line segments AB and CD intersect
def ccw(A,B,C):
    return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x)
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
def distance(A,B):
    return np.sqrt((A.x-B.x)**2 + (A.y-B.y)**2 + (A.z-B.z)**2)

def anti_collider():
    global waiting
    for veh in vehicle_list:
        if veh.number == vehicle_num:
            continue
        current_point = point(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)
        next_wp = point(tare_wp.point.x,tare_wp.point.y,tare_wp.point.z)
        #print((current_point, next_wp, veh.pos, veh.point))
        if intersect(current_point, next_wp, veh.pos, veh.point) and distance(current_point, veh.pos) > 2:
            if veh.priority > vehicle_veh.priority:
                continue
            if waiting:
                return
            waiting = True
            pub_tare_tog.publish(Bool(False))
            # make ugv of lower priority go to its left
            w = current_pose.pose.orientation.w
            r = 1
            x = current_point.x + r*np.cos(2*np.arccos(w)+np.pi/2)
            y = current_point.y + r*np.sin(2*np.arccos(w)+np.pi/2)
            z = current_point.z
            pub_wp.publish(PointStamped(header=Header(stamp=rospy.Time.now(),frame_id='map'), point=Point(x,y,z)))
            print('reversing')
            return
    waiting = False
    pub_tare_tog.publish(Bool(True))
    return

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('vehicle_manager')
    
    exploring_indices_publisher = rospy.Publisher("/Combined_Exploring_Indices", SubspaceArray, queue_size=10)
    covered_indices_publisher = rospy.Publisher("/Combined_Covered_Indices", SubspaceArray, queue_size=10)
    pub_poi = rospy.Publisher('/poi_in', PointStamped, queue_size=10)
    pub_vg = rospy.Publisher('/decoded_vgraph', Graph, queue_size=10)
    pub_kill = rospy.Publisher('/del_model_in', String, queue_size=5)
    pub_wp = rospy.Publisher('/way_point', PointStamped, queue_size=1)
    pub_tare_tog = rospy.Publisher('/toggle_wp', Bool, queue_size=5)
    pub_pose = rospy.Publisher('/pose_stamp', PoseStamped, queue_size=1)
    pub_keypose = rospy.Publisher('/other_keypose', NodeAndEdge, queue_size=1)
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.Subscriber('/state_estimation', Odometry, odom_cb, queue_size=1)
        rospy.Subscriber('/tare_way_point', PointStamped, twp_cb, queue_size=1)
        rospy.Subscriber('/far_way_point', PointStamped, twp_cb, queue_size=1)
        availtopics()
        updateVehicleStatus(vehicle_list)
        pub_covered_cell_indices(vehicle_list)
        pub_exploring_cell_indices(vehicle_list)
        anti_collider()
        rate.sleep()