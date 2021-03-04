import socket
from shapely.geometry import Point, point

class UDP_sender():
    def __init__(self, target):
        self.send_to_plotter = False
        self.send_to_gui = False
        
        if target == "GUI":
            # UDP connection to GUI
            self.send_to_gui = False
            self.udp_addr_gui = ("127.0.0.1", 5077)
            try:
                print("Init communication with external GUI")
                self.udp_send_sock_gui = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_send_sock_gui.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            except:
                print("Error during socket creation")
        
        elif target == "Plotter":
            print("Init UDP connection to GUI plotter")
            self.send_to_plotter = True
            self.udp_addr_plotter = ("127.0.0.1", 5078)
            try:
                print("Init communication with external GUI plotter")
                self.udp_send_sock_plotter = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_send_sock_plotter.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            except:
                print("Error during socket creation")


class Config():
    def __init__(self):
        print("Start configuration")
        
        
        self.log_everything = True # log more for debug

        self.status_send_cycle = 30 # Send status message every <status_send_cycle> time
        self.status_send_cycle_plotter = 30 # Send status message to plotter every <status_send_cycle> time
        
        self.left_border = -15
        self.right_border = 15
        
        # self.start_pos = Point(-1170, -981,-100)
        # self.goal_pos = Point(-1170, -522,-100)
        
        #down - > up
        self.start_pos = Point(-1039, -900,-100)
        self.goal_pos = Point(-454, -400, -100)
        
        # left-> right
        #self.start_pos = Point(-375,-880,-100)
        #self.goal_pos = Point(-367,-682,-100)
        self.max_distance_to_goal = 3  # [Meters] Drone stops when reaches max distance to goal
        self.speed = 4

        self.obst_timeout = 1.5     # [Sec] Timeout for the obstacle detection
        self.far_away_timeout = 1.5 # [Sec] If after this <var> seconds there is no obst. detection => report far_away
        self.m_line_time_delay = 5  # [Sec] start check

        self.close_range_thr = 7    # [Meters] Less than this thr => obstacle is NEAR. More than thr => obstacle is in MIDDLE range
        self.in_range_thr = 12      # [Meters] More than thr => obstacle is FAR
        self.max_m_line_dist = 4    # [Meters] If less than <var> meters to m-line => change back to GO_TO_LINE state

        self.correction_period = 1.5        # [Sec] Allow correcting course every <var> seconds while wall following
        self.reduce_range_timeout = 4       # [Sec] Stop trying to come back to the wall. Just fly to the target
        self.sharp_left_timeout = 1         # [Sec] Time period for completion of sharp left turn
        self.correction_obst_meet = -60     # [Deg] turn sharp <var> degrees when encountering obstacle in range in go to mode
        self.correction_obst_avoid = -90    # [Deg] turn sharp <var> degrees when encountering obstacle in range in ANY mode
        self.sharp_reduce_range = 90        # [Deg] turn sharp <var> degrees back to obstacle
        self.correction_return_to_obst = 20 # [Deg] turn <var> degrees towards or away from the wall to compensate the delta from the perfect distance
        self.collition_avoid_maneuver = 2   # [Sec] Allow sharp turn to avoid collition every <var> seconds
        print("Configuration complete!")