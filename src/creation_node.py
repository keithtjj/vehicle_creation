#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32MultiArray, Int32, Bool
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import time 
import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
import yaml
import os
import numpy as np
from sensor_msgs.msg import PointCloud2
# from octomap_msgs.msg import Octomap
# from octomap_msgs import binary_octomap
from visibility_graph_msg.msg import Graph
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from scipy.spatial import Voronoi, voronoi_plot_2d

import numpy as np
import random
import robot

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
best_vg = Graph()

poi_list = []

class point:
    def __init__(self,x=0,y=0,z=0):
        self.x = x
        self.y = y
        self.z = z
    def getxy(self):
        arr = [self.x,self.y]
        return arr

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
            if (topic.endswith("Exploring_subspaces")):
                self.exploring_sub = rospy.Subscriber(topic, Int32MultiArray, self.exploring_subspace_callback)
                print(self.name, "subscribed to indices")
            if (topic.endswith("Covered_subspaces")):
                self.covered_sub = rospy.Subscriber(topic, Int32MultiArray, self.covered_subspace_callback)
                print(self.name, "subscribed to indices")
            if (topic.endswith("Priority")):
                self.priority_sub = rospy.Subscriber(topic, Int32, self.priority_callback)
                print(self.name, "subscribed to priority")
            if (topic.endswith("tare_waypoint")):
                self.tare_sub = rospy.Subscriber(topic, PointStamped, self.multi_waypoint_callback)
                print(self.name, "subscribed to way points")
            if (topic.endswith("poi")):
                self.tare_sub = rospy.Subscriber(topic, PoseStamped, self.poi_callback)
                print(self.name, "subscribed to pois")
            if (topic.endswith("vgraph")):
                self.tare_sub = rospy.Subscriber(topic, Graph, self.vg_callback)
                print(self.name, "subscribed to vgraphs")
            if (topic.endswith("del_model")):
                self.tare_sub = rospy.Subscriber(topic, String, self.del_model_callback)
                print(self.name, "subscribed to model yeeter")

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
        # Store latest pose of the robot
        robot_id = odom_msg.child_frame_id
        pose = odom_msg.pose.pose
        # self.robot_poses[robot_id] = pose

        # Get the robot's linear and angular velocities
        self.linear_velocity = odom_msg.twist.twist.linear
        self.angular_velocity = odom_msg.twist.twist.angular

        # Extract the position and orientation data from the Odometry message
        self.position = odom_msg.pose.pose.position
        self.pos.x = odom_msg.pose.pose.position.x
        self.pos.y = odom_msg.pose.pose.position.y
        self.pos.z = odom_msg.pose.pose.position.z

        # Check if vehicle odometry interferes with local path
        if (self.number != vehicle_num):
            for i in range(len(self.local_path)):
                if self.isInsideCircularBoundary(self.position.x, self.position.y, self.position.z, self.obstacle_threshold, self.local_path[i]['position_x'], self.local_path[i]['position_y'], self.local_path[i]['position_z']):
                    self.redflag = 1
                else:
                    self.redflag = 0          

        # rospy.loginfo("Received odometry message: position = %s, orientation = %s", self.position, self.orientation)

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
        now = rospy.Time.now()
        if (waypoint.header.stamp.to_sec() > (now.to_sec()-5)):
            self.wp = waypoint
            self.point.x = waypoint.point.x
            self.point.y = waypoint.point.y
            self.point.z = waypoint.point.z   

    def poi_callback(self, poi):
        global poi_list
        r=5
        if poi.header.frame_id == 'test':
            return
        for po in poi_list:
            dx = po.pose.position.x - poi.pose.position.x
            dy = po.pose.position.y - poi.pose.position.y
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

class CoverageMapGenerator:
    def __init__(self, robot_name):

        self.robot_name = robot_name
        self.binary_map = None
        
        # Subscribe to the point cloud topic for this robot
        self.point_cloud_sub = rospy.Subscriber(f"/{self.robot_name}/point_cloud", PointCloud2, self.process_point_cloud)

        # Set up publisher for the coverage map
        self.coverage_map_pub = rospy.Publisher(f"/{self.robot_name}/coverage_map", Octomap, queue_size=10)
    

    def process_point_cloud(self, point_cloud):
        # Convert PointCloud2 message to PointCloud2 iterator
        points = point_cloud2.read_points(point_cloud, field_names=("x", "y", "z"))

        # Create octree object
        octree = octomap.OcTree(0.1)

        # Add points to octree
        for point in points:
            octree.updateNode(point[0], point[1], point[2], True)

        # Convert octree to binary octomap message
        binary_map = octomap_msgs.binary_octomap.Octomap()
        binary_map.header.frame_id = point_cloud.header.frame_id
        binary_map.header.stamp = rospy.Time.now()
        octomap_msgs.binary_octomap.binaryMapToMsg(octree, binary_map)

        # Publish coverage map
        if self.binary_map is not None:
            self.coverage_map_pub.publish(self.binary_map)

class BufferedVoronoiCell:
    def __init__(self, pos, rs):
        self.pos = pos
        self.rs = rs
        self.voronoi = None
        self.buffered_vertices = []
        self.buffered_edges = []
        self.buffered_regions = []
        self.compute_buffered_voronoi_cell()
    
        # Publishers for the regions and markers
        self.regions_pub = rospy.Publisher('regions', list, queue_size=10)
        self.marker_pub = rospy.Publisher('markers', Marker, queue_size=10)


    def compute_buffered_voronoi_cell(self):
        # Compute the standard Voronoi cell
        self.voronoi = Voronoi(self.pos)
        vertices = self.voronoi.vertices
        ridges = self.voronoi.ridge_points
        regions = self.voronoi.regions

        # Compute the buffered Voronoi cell
        for i, region in enumerate(regions):
            if len(region) == 0 or -1 in region:
                continue
            new_region = []
            for j, rid in enumerate(region):
                p1, p2 = ridges[rid]
                if p1 == -1 or p2 == -1:
                    break
                v1, v2 = vertices[p1], vertices[p2]
                mid = (v1 + v2) / 2
                dir_vec = v2 - v1
                dir_unit_vec = dir_vec / np.linalg.norm(dir_vec)
                normal_vec = np.array([-dir_unit_vec[1], dir_unit_vec[0]])
                dist = np.linalg.norm(self.pos[i] - mid)
                buffered_vertex = mid + self.rs * normal_vec
                self.buffered_vertices.append(buffered_vertex)
                new_region.append(len(self.buffered_vertices) - 1)
                if ridges[rid][1] != -1:
                    self.buffered_edges.append([len(self.buffered_vertices) - 1,
                                                len(self.buffered_vertices)])
                else:
                    self.buffered_edges.append([len(self.buffered_vertices) - 1,
                                                new_region[0]])
            self.buffered_regions.append(new_region)

    def plot_buffered_voronoi_cell(self, ax=None):
        if ax is None:
            fig, ax = plt.subplots()
        if self.voronoi is not None:
            voronoi_plot_2d(self.voronoi, ax=ax, show_vertices=False, line_colors='orange')
        for i, region in enumerate(self.buffered_regions):
            vertices = [self.buffered_vertices[v] for v in region]
            if len(vertices) > 2:
                ax.fill(*zip(*vertices), alpha=0.2)
            for edge in self.buffered_edges:
                x = [self.buffered_vertices[edge[0]][0], self.buffered_vertices[edge[1]][0]]
                y = [self.buffered_vertices[edge[0]][1], self.buffered_vertices[edge[1]][1]]
                ax.plot(x, y, 'k-', alpha=0.5)

    # Publishes a marker for visualization in RViz
    def publish_marker(self, polygon, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.POLYGON
        marker.action = Marker.ADD
        marker.points = [PointStamped(point=point) for point in polygon]
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color = color
        marker.lifetime = rospy.Duration.from_sec(1)
        self.marker_pub.publish(marker)


def updategraph(vehicles):
    rs = 0.5
    robot_bvcs = []
    for vehicle in vehicles:
        bvc = BufferedVoronoiCell(vehicle.position, vehicle.linear_velocity)
        robot_bvcs.append(bvc)


def generate_vehicle(topic, vehicle_num, name):
    if name not in vehicle_name_list:
        # Create a new instance of the Vehicle class with the specified topic, number, and name
        vehicle = Vehicle(topic, vehicle_num, name)

        #Add Vehicle Created to list of created vehicles
        vehicle_name_list.append(vehicle.name)
        vehicle_list.append(vehicle)  
        vehicle_index_list.append(int(vehicle.number))

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
    
    # # Create Coverage Map for each Vehicle
    # for robot_name in vehicle_name_list:
    #     if robot_name not in generators_name:
    #         generator = CoverageMapGenerator(robot_name)
    #         generators.append(generator)
    #         generators_name.append(robot_name)

def updateVehicleStatus(vehicles):
    for vehicle in vehicles:
        # print(vehicle.name)
        vehicle.update_vehicle()
        if vehicle.redflag == 1:
            print(vehicle.name + " is invading personal space")

def pub_exploring_cell_indices(vehicles):
    exploring_array = Int32MultiArray()
    exploring_cell_indices = []
    for vehicle in vehicles:
        if (vehicle.number != vehicle_num):
            for i in vehicle.exploring_indices:
                exploring_cell_indices.append(int(i))
    exploring_array.data = exploring_cell_indices
    exploring_indices_publisher.publish(exploring_array)

def pub_covered_cell_indices(vehicles):
    cover_array = Int32MultiArray()
    covered_cell_indices = []
    for vehicle in vehicles:
        if (vehicle.number != vehicle_num):
            for i in vehicle.covered_indices:
                covered_cell_indices.append(int(i))
    cover_array.data = covered_cell_indices
    covered_indices_publisher.publish(cover_array)
    
def multi_waypoint_sub():
    #This function adds a new key-value pair for each new availability topic in ROS using rospy.

    # Get a list of all published topics
    topics = rospy.get_published_topics()

    # Filter the list of topics to only include those with the header "/Availability"
    tare_topics = [topic_name for topic_name, _ in topics if (topic_name.find('tare_waypoint')!=(-1))]

    #Dictionary for availability topics
    tare_dict = {}

    #Filter Topic name for numbers and name vehicle according to number
    for topic_name in tare_topics:
        split = topic_name.split('/')
        for elem in split:
            if (elem.isnumeric() and (elem != vehicle_num)):
                robot_name = f"robot_{elem}" # Generate a key name like "vehicle_1", "vehicle_2", etc.
                if robot_name not in tare_dict:
                    tare_dict[robot_name] = topic_name
                    #check if vehicle has generated object, if none create object
                    if robot_name not in tare_name_list:
                        planner = tare(robot_name,topic_name)
                        tare_list.append(planner)
                        tare_name_list.append(planner.id)

def collision_avoidance(vehicle_list,vehicle_name_list):
    flag = 0
    robots = [robot.Robot(i) for i in range(len(vehicle_name_list))]

    for num, vehicle in enumerate(vehicle_list):
        if (vehicle.planner != None):
            print(vehicle.pos.getxy())
            robots[num].set_bvc(vehicle.pos.getxy(), safe_rad, corners)
            robots[num].set_goal(vehicle.planner.point.getxy())
            flag = 1
        else:
            flag = 0

    if (flag == 1):
        for loop in range(10000):
            # print loop
            # all robots re-compute BVC
            for i , name in enumerate(vehicle_name_list):
                all_pos = np.zeros((2,5))
                for r in range(len(vehicle_name_list)):
                    all_pos[:,r,None] = robots[r].get_pos() # none is used to preserve dimensionality
                    print(all_pos)

                bb = [x for x in vehicle_index_list if x != i]
                bb_array = np.array(bb) # this is the IDs of neighboring robots
                
                # extract own pos for robot i, as well as all other neighbor robots
                own_pos = all_pos[:,i]
                nbr_pos = all_pos[:,bb_array]
                
                robots[i].cell.update_bvc(own_pos, nbr_pos, bb_array)
                robots[i].mem_nbr_dist(nbr_pos, bb_array)
                robots[i].cell.plot_bvc()

            # all robots do geometric solution
            for rbt in robots:
                closest = rbt.bvc_find_closest_to_goal()
                print(closest)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('vehicle_manager')
    
    exploring_indices_publisher = rospy.Publisher("/Combined_Exploring_Indices", Int32MultiArray, queue_size=10)
    covered_indices_publisher = rospy.Publisher("/Combined_Covered_Indices", Int32MultiArray, queue_size=10)
    pub_poi = rospy.Publisher('/poi_in', PoseStamped, queue_size=10)
    pub_vg = rospy.Publisher('/decoded_vgraph', Graph, queue_size=10)
    pub_kill = rospy.Publisher('/del_model', String, queue_size=5)

    r = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        availtopics()
        updateVehicleStatus(vehicle_list)
        pub_covered_cell_indices(vehicle_list)
        pub_exploring_cell_indices(vehicle_list)
        collision_avoidance(vehicle_list,vehicle_name_list)
        rospy.sleep(1)





