import socket

class Config():
    def __init__(self):
        print("Start configuration")
        # UDP connection
        self.send_to_gui = False
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
        
        self.left_border = -30
        self.right_border = 30
        
        # self.start_pos = (-1170, -981,-100)
        # self.goal_pos = (-1170, -522,-100)
        
        #down - > up
        # self.start_pos = (-414, -860,-100)
        # self.goal_pos = (-402, -727, -100)
        
        # left-> right
        self.start_pos = (-375,-930,-100)
        self.goal_pos = (-367,-682,-100)
        self.speed = 5

        self.obst_timeout = 3 # [Sec] timeout for the obstacle detection
        self.close_range_thr = 3 # [Meters] Less than this thr => obstacle is NEAR. More than thr => obstacle is IN_RANGE
        self.in_range_thr = 10 # More than thr => obstacle is FAR
        print("Configuration complete!")