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
from functools import total_ordering
import math
import json
import logging

# Init logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

fmt = '%(asctime)s %(message)s'
#date_fmt = "%H:%M:%S"
formatter = logging.Formatter(fmt)#, date_fmt)

file_handler = logging.FileHandler('bug2.log', mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

stream_handler = logging.StreamHandler()
stream_handler.setLevel(logging.INFO)
stream_handler.setFormatter(formatter)
logger.addHandler(stream_handler)

logger.propagate = False

logger.info("Log started")



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
    def __init__(self, config, udp_sender_gui, client):
        self.config = config
        self.udp_sender_gui = udp_sender_gui
        self.state = bug2_state.GO_TO_POINT # initial state
        self.start_pos = config.start_pos
        self.goal_pos = config.goal_pos
        self.speed = config.speed
        self.client = client
        self.obstacles = ObstaclesDirections(config, udp_sender_gui)
        self.desired_azimuth = self.calc_azimuth_from_two_points(self.start_pos, self.goal_pos) # Drone navigation
        self.drone_position = None          # holds real drone position
        self.drone_azimuth = None           # holds real drone azimuth
        now = time.time()
        self.last_state_change_time = now       # last state change
        self.last_nonzero_correction_time = now # last course correction
        self.last_sharp_turn_avoid_coll = now   # last sharp turn (collision avoidance)
        self.last_goto_state_time = now         # last goto state was active
        self.sharp_left_init_time = now - 2*self.config.sharp_left_timeout # last sharp left time

        
        

    def run_iteration(self, lidar_relative_drone, lidar_world_frame, drone_position, drone_azimuth):
        self.drone_azimuth = drone_azimuth
        self.drone_position = drone_position
        points = np.array(lidar_relative_drone.points, dtype=np.dtype('f4'))
        #print(len(points), points)
        distance_to_m_line = self.distance_to_line(drone_position)
        #print("distance to m-line: {:.2f}".format(distance_to_m_line))

        if int(len(points)) > 1:
            # Obstacle detected
            front = points[0]
            side = points[1]
            dist_to_obst = np.sqrt(front**2+side**2)
            angle = np.arctan2(side,front)
            self.obstacles.update_description(np.degrees(angle), dist_to_obst)
        
        now = time.time()

        distance_to_goal = self.distance_to_goal()
        if distance_to_goal < self.config.max_distance_to_goal:
            self.client.flyToPosition(0, 0, 0, 0)
            self.change_state(bug2_state.FINISH)

        if now - self.last_goto_state_time > self.config.m_line_time_delay and distance_to_m_line < self.config.max_m_line_dist:
            self.change_state(bug2_state.GO_TO_POINT)

        # Go to state
        if self.state == bug2_state.GO_TO_POINT:
            self.last_goto_state_time = now
            if self.obstacles.sectors[obst_direction.FRONT].get_range() <= obst_range.MIDDLE:
                
                self.correct_desired_azimuth_and_fly(self.config.correction_obst_meet) # Sharp left
                # Change state
                self.sharp_left_init_time = now
                self.change_state(bug2_state.WALL_FOLLOW_SET_COURSE)
            else:
                
                # Continue going to the point
                self.desired_azimuth = self.calc_azimuth_from_two_points(self.drone_position, self.goal_pos)
                #print("Drone: {} Goal {} az {}".format(self.drone_position, self.goal_pos, self.desired_azimuth))
                self.correct_desired_azimuth_and_fly(0)

        # Transfer to wall follow state
        elif self.state == bug2_state.WALL_FOLLOW_SET_COURSE:
            if now - self.sharp_left_init_time > self.config.sharp_left_timeout:
                self.change_state(bug2_state.WALL_FOLLOW_HOLD_RANGE)
        
        # Wall follow hold range state
        elif self.state == bug2_state.WALL_FOLLOW_HOLD_RANGE:
            if (self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.NEAR
               and now - self.last_nonzero_correction_time > self.config.correction_period):
                
                self.correct_desired_azimuth_and_fly(-self.config.correction_return_to_obst) # correct range once and change to suitable state
                self.change_state(bug2_state.WALL_FOLLOW_INCREASE_RANGE)
            
            elif self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.MIDDLE:
            
                self.correct_desired_azimuth_and_fly(0) # Stay on course
            
            elif (self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.FAR
                and now - self.last_nonzero_correction_time > self.config.correction_period):
                
                self.correct_desired_azimuth_and_fly(self.config.correction_return_to_obst) # correct range once and change to suitable state
                self.change_state(bug2_state.WALL_FOLLOW_REDUCE_RANGE)
            
            elif (self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.FAR_AWAY
                and now - self.last_nonzero_correction_time > self.config.correction_period):
                
                self.correct_desired_azimuth_and_fly(self.config.sharp_reduce_range)
                self.change_state(bug2_state.WALL_FOLLOW_SHARP_REDUCE_RANGE)

        # Wall follow increase range state
        elif self.state == bug2_state.WALL_FOLLOW_INCREASE_RANGE:
            if (self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.NEAR
                and now - self.last_nonzero_correction_time > self.config.correction_period):
                self.correct_desired_azimuth_and_fly(-self.config.correction_return_to_obst) # increase even more

            elif self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.MIDDLE:
                self.correct_desired_azimuth_and_fly(self.config.correction_return_to_obst) # Correct the course back. You are on perfect range.
                self.change_state(bug2_state.WALL_FOLLOW_HOLD_RANGE) # come back to hold range state
            
            elif (self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.FAR
                and now - self.last_nonzero_correction_time > self.config.correction_period):
                self.correct_desired_azimuth_and_fly(self.config.correction_return_to_obst) # stop increasing. start reducing
                self.change_state(bug2_state.WALL_FOLLOW_REDUCE_RANGE)

            elif (self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.FAR_AWAY
                and now - self.last_nonzero_correction_time > self.config.correction_period):
                self.correct_desired_azimuth_and_fly(self.config.sharp_reduce_range)
                self.change_state(bug2_state.WALL_FOLLOW_SHARP_REDUCE_RANGE)

        # Wall follow reduce range state
        elif self.state == bug2_state.WALL_FOLLOW_REDUCE_RANGE:
            if (self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.NEAR
                and now - self.last_nonzero_correction_time > self.config.correction_period):
                self.correct_desired_azimuth_and_fly(-self.config.correction_return_to_obst) # increase range
                self.change_state(bug2_state.WALL_FOLLOW_INCREASE_RANGE) # increase range state

            elif self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.MIDDLE:
                self.correct_desired_azimuth_and_fly(-self.config.correction_return_to_obst) # Correct the course back. You are on perfect range.
                self.change_state(bug2_state.WALL_FOLLOW_HOLD_RANGE) # come back to hold range state
            
            elif (self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.FAR
                and now - self.last_nonzero_correction_time > self.config.correction_period):

                self.correct_desired_azimuth_and_fly(self.config.correction_return_to_obst) # reduce even more

            elif (self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.FAR_AWAY
                and now - self.last_nonzero_correction_time > self.config.correction_period):
                self.correct_desired_azimuth_and_fly(self.config.sharp_reduce_range)
                self.change_state(bug2_state.WALL_FOLLOW_SHARP_REDUCE_RANGE)

        # Wall follow reduce range sharply
        elif self.state == bug2_state.WALL_FOLLOW_SHARP_REDUCE_RANGE:
            if self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.NEAR:
                self.correct_desired_azimuth_and_fly(0) # Stay on course
                self.change_state(bug2_state.WALL_FOLLOW_INCREASE_RANGE) # come back to hold range state

            elif self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.MIDDLE:
                self.correct_desired_azimuth_and_fly(-self.config.correction_return_to_obst) # Correct the course back. You are on perfect range.
                self.change_state(bug2_state.WALL_FOLLOW_HOLD_RANGE) # come back to hold range state
            
            elif self.obstacles.sectors[obst_direction.RIGHT].get_range() == obst_range.FAR:
                self.correct_desired_azimuth_and_fly(0) # Stay on course
                self.change_state(bug2_state.WALL_FOLLOW_REDUCE_RANGE) # come back to hold range state

            elif now - self.last_nonzero_correction_time > self.config.reduce_range_timeout:
                self.change_state(bug2_state.GO_TO_POINT) # Stop trying to return to the obstacle
            
        # Avoid front collition
        if (self.obstacles.sectors[obst_direction.FRONT].get_range() == obst_range.NEAR
            and now - self.last_sharp_turn_avoid_coll > self.config.collition_avoid_maneuver):
            
            self.correct_desired_azimuth_and_fly(self.config.correction_obst_avoid)
            logger.info("Avoid collition from FRONT! Sharp left!")
            self.last_sharp_turn_avoid_coll = now

            if self.state != bug2_state.GO_TO_POINT: # every WALL_FOLLOW state will turn to WALL_FOLLOW_HOLD_RANGE
                self.change_state(bug2_state.WALL_FOLLOW_HOLD_RANGE)

        

        self.obstacles.print_state(self.drone_azimuth, self.desired_azimuth, self.state, distance_to_goal, distance_to_m_line)


    def change_state(self, new_state):
        if new_state != self.state:
            #print("State: {} -> {}".format(self.state.name, new_state.name))
            #logger.info("{}  ( {} )".format(self.obstacles.get_obst_string(), ObstaclesDirections.get_state_as_string(new_state)))
            self.state = new_state
            self.last_state_change_time = time.time()
    
    def correct_desired_azimuth_and_fly(self, correction_deg):
        if correction_deg < -0.01 or correction_deg > 0.01: # corection != 0
            self.last_nonzero_correction_time = time.time()
            new_azimuth = (self.desired_azimuth + correction_deg + 360 ) % 360 # keep azimuth in range (0,360)
            logger.info("az {:5.1f} delta {:5.1f} new {:5.1f}".format(self.desired_azimuth, correction_deg, new_azimuth))
        else:
            new_azimuth = self.desired_azimuth # remain untoched
            
        self.desired_azimuth = new_azimuth
        
        #print("desired az {:.2f}".format(self.desired_azimuth))
        curr_goal_pos = self.calc_desired_wp_from_azimuth(self.drone_position, self.desired_azimuth)

        #print("fly to: {} {} {}".format(curr_goal_pos.x, curr_goal_pos.y, curr_goal_pos.z))
        self.client.flyToPosition(curr_goal_pos.x, curr_goal_pos.y, curr_goal_pos.z, self.speed)

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
    
    def distance_to_goal(self):
        p1 = Point(self.drone_position)
        p2 = Point(self.goal_pos)

        return math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))

    
    def calc_azimuth_from_two_points(self, p0, p1):
        azimuth_rad = math.atan2(p1.y-p0.y, p1.x-p0.x)
        return (math.degrees(azimuth_rad) + 360 ) % 360
    
    def calc_desired_wp_from_azimuth(self, self_loc, azimuth):
        ''' Create new way point that is _ meters away of the self_loc at the direction of azimuth'''
        d = 20 # distance of the waypoint
        
        x = self_loc.x + d * math.cos(math.radians(azimuth))
        y = self_loc.y + d * math.sin(math.radians(azimuth))
        return Point(x, y, self.goal_pos.z)

class ObstaclesDirections():
    ''' Handles the range report for all the sectors (FRONT/LEFT/etc.) '''
    def __init__(self, config, udp_sender_gui):
        self.config = config
        self.udp_sender_gui = udp_sender_gui
        self.not_sent_statuses = 0
        
        # Initialize obstacle descriptors for each sector with timed out detection times  
        self.sectors = {obst_direction.LEFT:  ObstacleDescriptor(config),
                        obst_direction.FRONT: ObstacleDescriptor(config),
                        obst_direction.RIGHT: ObstacleDescriptor(config)}
    
    def update_description(self, angle, distance):
        #logger.info("angle: {}".format(angle))
        self.sectors[self.angle_to_direction(angle)].update_description(time.time(), distance)

    def angle_to_direction(self, angle):
        if angle < self.config.left_border:
            return obst_direction.LEFT
        elif angle < self.config.right_border:
            return obst_direction.FRONT
        else:
            return obst_direction.RIGHT

    def print_state(self, drone_azimuth, desired_azimuth, drone_state, distance_to_goal, distance_to_m_line):
        drone_state_str = ObstaclesDirections.get_state_as_string(drone_state)
        obst_string = self.get_obst_string()
        status_msg = "{}\n{}\nazimuth : {:.1f}\ndesired : {:.1f}\nto goal : {:.1f}\nto mline: {:.1f}".format(drone_state_str, obst_string, drone_azimuth, desired_azimuth, distance_to_goal, distance_to_m_line)
        if self.config.log_everything:
            logger.info("{}  ( {} ) az {:.1f} ({:.1f})   goal {:.1f} mline {:.1f}".format(obst_string, drone_state_str, drone_azimuth, desired_azimuth, distance_to_goal, distance_to_m_line))
        # Send status over UDP
        if self.udp_sender_gui.send_to_gui and self.not_sent_statuses >= self.config.status_send_cycle:
            self.not_sent_statuses = 0
            msg = str.encode(json.dumps(status_msg))
            self.udp_sender_gui.udp_send_sock_gui.sendto(msg, self.udp_sender_gui.udp_addr_gui)
        self.not_sent_statuses += 1
    
    @staticmethod
    def get_state_as_string(state):
        if state == bug2_state.GO_TO_POINT:
            return "        ^^^        "
        elif state == bug2_state.WALL_FOLLOW_SET_COURSE:
            return " <<<<< <<<<<       "
        elif state == bug2_state.WALL_FOLLOW_HOLD_RANGE:
            return "        ---        "
        elif state == bug2_state.WALL_FOLLOW_REDUCE_RANGE:
            return "              >>>  "
        elif state == bug2_state.WALL_FOLLOW_INCREASE_RANGE:
            return "  <<<              "
        elif state == bug2_state.WALL_FOLLOW_SHARP_REDUCE_RANGE:
            return "       >>>>> >>>>> "
        elif state == bug2_state.FINISH:
            return " OOOOO OOOOO OOOOO "
        else:
            return "       WTF         "
    
    def get_obst_string(self):
        left_range  = self.sectors[obst_direction.LEFT].get_range_as_string()
        front_range = self.sectors[obst_direction.FRONT].get_range_as_string()
        right_range = self.sectors[obst_direction.RIGHT].get_range_as_string()

        return "[{}|{}|{}]".format(left_range, front_range, right_range)


    
class ObstacleDescriptor():
    ''' Handles the range report for one sector (FRONT/LEFT/etc.) '''
    def __init__(self, config):
        self.config = config
        self.last_detection_time = time.time()-2*config.obst_timeout
        self.range = obst_range.FAR_AWAY # init as a timed out obstacle

    def update_description(self, new_detection_time, distance):
        new_range = self.distance_to_range(distance)
        if new_range <= self.range: # always update with the closest range
            self.range = new_range
            self.last_detection_time = new_detection_time
        elif new_detection_time - self.last_detection_time > self.config.obst_timeout: # update with more distant range only after timeout
            self.range = new_range
            self.last_detection_time = new_detection_time # TODO thing about this.. may cause slow detection of the distant ranges..
           
    def distance_to_range(self, distance):
        if distance < self.config.close_range_thr:
            return obst_range.NEAR
        elif distance < self.config.in_range_thr:
            return obst_range.MIDDLE
        else:
            return obst_range.FAR

    def get_range(self):
        # Check if obstacle is not timed out
        if time.time() - self.last_detection_time > self.config.far_away_timeout: # TODO recheck the FAR_AWAY logic...
            self.range = obst_range.FAR_AWAY

        return self.range
    
    def get_range_as_string(self):
        range = self.get_range()
        if range == obst_range.NEAR:
            return "CLOSE"
        elif range == obst_range.MIDDLE:
            return "  V  "
        elif range == obst_range.FAR:
            return " FAR "
        elif range == obst_range.FAR_AWAY:
            return " FAR!"
        else:
            return "?WTF?"

class bug2_state(Enum):
    GO_TO_POINT = 0
    WALL_FOLLOW_SET_COURSE = 1
    WALL_FOLLOW_HOLD_RANGE = 2
    WALL_FOLLOW_REDUCE_RANGE = 3
    WALL_FOLLOW_INCREASE_RANGE = 4
    WALL_FOLLOW_SHARP_REDUCE_RANGE = 5
    FINISH = 6

class obst_direction(Enum):
    LEFT = 0
    FRONT = 1
    RIGHT = 2

@total_ordering
class obst_range(Enum):
    NEAR = 0
    MIDDLE = 1
    FAR = 2
    FAR_AWAY = 3 # Timed out
    def __lt__(self, other): # implementation of < operator
       if self.__class__ is other.__class__:
           return self.value < other.value
       return NotImplemented



        
    
    
            


    


