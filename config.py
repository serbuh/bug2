import socket
from shapely.geometry import Point, point

class Config():
    def __init__(self):
        print("Start configuration")
        # UDP connection
        self.send_to_gui = True
        self.udp_addr = ("127.0.0.1", 5077)
        try:
            if self.send_to_gui:
                print("Init communication with external GUI")
                self.udp_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            else:
                self.udp_send_sock = None
        except:
            print("Error during socket creation")
        
        self.status_send_cycle = 30 # Send status message every <status_send_cycle> time 

        self.left_border = -30
        self.right_border = 30
        
        # self.start_pos = Point(-1170, -981,-100)
        # self.goal_pos = Point(-1170, -522,-100)
        
        #down - > up
        # self.start_pos = Point(-414, -860,-100)
        # self.goal_pos = Point(-402, -727, -100)
        
        # left-> right
        self.start_pos = Point(-375,-880,-100)
        self.goal_pos = Point(-367,-682,-100)
        self.speed = 5

        self.obst_timeout = 1      # [Sec] Timeout for the obstacle detection
        self.close_range_thr = 6   # [Meters] Less than this thr => obstacle is NEAR. More than thr => obstacle is IN_RANGE
        self.in_range_thr = 12     # [Meters] More than thr => obstacle is FAR

        self.correction_period = 1 # [Sec] Allow correcting course every <var> seconds while wall following
        self.correction_obst_meet = -90 # [Deg] turn <var> degrees when encountering obstacle in range in go to mode
        self.correction_return_to_obst = 20 # [Deg] turn <var> degrees towards or away from the wall to compensate the delta from the perfect distance
        print("Configuration complete!")