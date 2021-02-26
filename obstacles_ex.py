from DroneClient import DroneClient
import time
import airsim.utils
#from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString 
from shapely.geometry import Point, point
import numpy as np
import pandas 
import matplotlib.pyplot as plt
from enum import Enum
import math
import json


def ccw_sort(p):
        """Sort given polygon points in CCW order"""
        p = np.array(p)
        mean = np.mean(p,axis=0)
        d = p-mean
        s = np.arctan2(d[:,0], d[:,1])
        return p[np.argsort(s),:]

def sort_polygon(coords):
    x=coords
    lx=[]
    for i in range(int(len(x)/2)):
        lx.append((x[2*i], x[2*i+1]))
    sorted_coords_list =list(ccw_sort(lx))
    return sorted_coords_list

def obstacles_ex(obs_cordinates):
    obs_dic = {}
    for line in obs_cordinates:
        if line[4] not in obs_dic.keys(): #  obs type
            obs_dic[line[4]] = []
        obs_dic[line[4]].append(line[1]) # x- coord
        obs_dic[line[4]].append(line[2])# y- coord

    obstacles = []
    for key in obs_dic.keys():
        sorted = sort_polygon(obs_dic[key])   
        polygon = Polygon(sorted)
        obstacles.append(polygon)
    
    return obstacles

def rotation(x,y,z):
    rx= np.array([[1, 0,0], [0, np.cos(x), np.sin(x)], [0, -np.sin(x), np.cos(x)]])
    ry = np.array([ [np.cos(y),0, -np.sin(y)],  [0, 1,0],    [np.sin(y),0, np.cos(y)]])  
    rz = np.array([ [np.cos(z),np.sin(z), 0], [-np.sin(z), np.cos(z),0],  [0, 0,1] ]) 
    r= ry.dot(rz)
    r= rx.dot(r)
    return r


def lidar_points(lidar_relative_drone,rotation_matrix): # rotation * pos_relative_drone
    rdotp=np.array([])
    points = np.array(lidar_relative_drone.points, dtype=np.dtype('f4'))
    if int(len(points)) > 1:
        points = np.array([points[0],points[1],points[2]])
        rdotp = rotation_matrix.dot(points)
        return rdotp
    else:
        return None # No obstacles detected
   


def get_lidar_world_frame(lidar_relative_drone,res ):
    rotation_matrix = rotation(res.orientation.x_rad,  res.orientation.y_rad, -res.orientation.z_rad)
    rdotp=lidar_points(lidar_relative_drone,rotation_matrix)

    if rdotp is not None:
        lidar_world_frame    = np.array([res.pos.x_m, res.pos.y_m, res.pos.z_m])+rdotp
        return lidar_world_frame   
    else:
        return None # No obstacles detected



class bug2():
    def __init__(self, config, client):
        self.state = bug2_state.GO_TO_POINT # initial state
        self.start_pos = config.start_pos
        self.goal_pos = config.goal_pos
        self.speed = config.speed
        self.client = client
        self.state_time_counter = 0
        self.obstacles = ObstaclesDirections(config)
        

    def run_iteration(self, lidar_relative_drone, lidar_world_frame, drone_position, drone_azimuth):
        self.drone_azimuth = drone_azimuth
        points = np.array(lidar_relative_drone.points, dtype=np.dtype('f4'))
        #print(len(points), points)
        dist_to_m_line = self.distance_to_line(drone_position)
        #print("distance to m-line: {:.2f}".format(dist_to_m_line))
        #self.client.flyToPosition(100, 200, self.goal_pos[2], self.speed)
        
        if int(len(points)) > 1:
            # Obstacle detected
            front = points[0]
            side = points[1]
            dist_to_obst = np.sqrt(front**2+side**2)
            angle = np.arctan2(side,front)
            self.obstacles.update_description(np.degrees(angle), dist_to_obst)
            
        self.obstacles.print_state(self.drone_azimuth)

        if self.state == bug2_state.GO_TO_POINT:
            if (self.obstacles.sectors[obst_direction.FRONT].get_range() == obst_range.IN_RANGE 
                or self.obstacles.sectors[obst_direction.FRONT].get_range() == obst_range.NEAR):
                    self.change_state(bug2_state.WALL_FOLLOW)
            self.client.flyToPosition(self.goal_pos[0], self.goal_pos[1], self.goal_pos[2], self.speed)
                
            
        elif self.state == bug2_state.WALL_FOLLOW:
            self.client.flyToPosition(self.start_pos[0], self.start_pos[1], self.start_pos[2], 0)
            if self.state_time_counter > 5 and dist_to_m_line < 3: # TODO play with those params
                self.change_state(bug2_state.GO_TO_POINT)
        
        self.state_time_counter += 1

    def change_state(self, new_state):
        if new_state != self.state:
            print("State: {} -> {}".format(self.state.name, new_state.name))
            self.state = new_state
            self.state_time_counter = 0
            
    def distance_to_line(self, p0):
        # p0 is the current position
        # p1 and p2 points define the line
        p1 = Point(self.start_pos)
        p2 = Point(self.goal_pos)
        # distance equation
        up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
        lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
        distance = up_eq / lo_eq

        return distance

class ObstaclesDirections():
    ''' Handles the range report for all the sectors (FRONT\LEFT\etc.) '''
    def __init__(self, config):
        self.config = config
        self.not_sent_statuses = 0
        
        # Initialize obstacle descriptors for each sector with timed out detection times  
        self.sectors = {obst_direction.LEFT:  ObstacleDescriptor(config),
                        obst_direction.FRONT: ObstacleDescriptor(config),
                        obst_direction.RIGHT: ObstacleDescriptor(config)}
    
    def update_description(self, angle, distance):
        self.sectors[self.angle_to_direction(angle)].update_description(time.time(), distance)

    def angle_to_direction(self, angle):
        if angle < self.config.left_border:
            return obst_direction.LEFT
        elif angle < self.config.right_border:
            return obst_direction.FRONT
        else:
            return obst_direction.RIGHT

    def print_state(self, drone_azimuth):
        left_range  = self.sectors[obst_direction.LEFT].get_range_as_string()
        front_range = self.sectors[obst_direction.FRONT].get_range_as_string()
        right_range = self.sectors[obst_direction.RIGHT].get_range_as_string()
        
        status_msg = "[{}|{}|{}] az {:.1f}".format(left_range, front_range, right_range, drone_azimuth)
        #print(status_msg)
        # Send status over UDP
        if self.config.send_to_gui and self.not_sent_statuses >= self.config.status_send_cycle:
            self.not_sent_statuses = 0
            msg = str.encode(json.dumps(status_msg))
            self.config.udp_send_sock.sendto(msg, self.config.udp_addr)
        self.not_sent_statuses += 1
    
class ObstacleDescriptor():
    ''' Handles the range report for one sector (FRONT\LEFT\etc.) '''
    def __init__(self, config):
        self.config = config
        self.last_detection_time = time.time()-2*config.obst_timeout
        self.range = obst_range.NOT_IN_RANGE # init as a timed out obstacle

    def update_description(self, last_detection_time, distance):
        self.last_detection_time = last_detection_time
        self.range = self.distance_to_range(distance)
    
    def distance_to_range(self, distance):
        if distance < self.config.close_range_thr:
            return obst_range.NEAR
        elif distance < self.config.in_range_thr:
            return obst_range.IN_RANGE
        else:
            return obst_range.FAR

    def get_range(self):
        # Check if obstacle is not timed out
        if time.time() - self.last_detection_time > self.config.obst_timeout:
            self.range = obst_range.NOT_IN_RANGE

        return self.range
    
    def get_range_as_string(self):
        range = self.get_range()
        if range == obst_range.NEAR:
            return "#####"
        elif range == obst_range.IN_RANGE:
            return " ### "
        elif range == obst_range.FAR:
            return "  #  "
        elif range == obst_range.NOT_IN_RANGE:
            return "     "
        else:
            return "?WTF?"

class bug2_state(Enum):
    GO_TO_POINT = 0
    WALL_FOLLOW = 1

class obst_direction(Enum):
    LEFT = 0
    FRONT = 1
    RIGHT = 2

class obst_range(Enum):
    NEAR = 0
    IN_RANGE = 1
    FAR = 2
    NOT_IN_RANGE = 3 # Timed out



        
    
    
            


    


