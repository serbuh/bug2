import sys
import socket
import json
import select

# plotter imports
from config import Config
import pandas
import numpy as np
from plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString
import coordinate_convertions

config = Config() # load config

if sys.version_info[0] == 2:
    import Tkinter as tk # for python2
else:
    import tkinter as tk # for python3

class UDP():
    def __init__(self, udp_receiver):
        self.udp_receiver = udp_receiver
        self.bind_recv(udp_receiver)
        
    def bind_recv(self, udp_receiver):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(udp_receiver)
        self.sock.setblocking(0)

    def recv(self):
        try:
            readable, writable, exceptional = select.select([self.sock], [], [self.sock], 0)
            for s in readable:
                if s is self.sock:
                    recv_data, address = s.recvfrom(4096)
                    if isinstance(recv_data, bytes):
                        recv_data = recv_data.decode() # recvfrom returnes bytes in python3. json.loads() receives str.
                    return json.loads(recv_data)
        except BlockingIOError as e:
            print("Socket error: {}".format(e))

    def recv_list(self):
        try:
            chunk_list = []
            new_data_available = True
            while new_data_available:
                readable, writable, exceptional = select.select([self.sock], [], [self.sock], 0)
                if len(readable) == 0:
                    new_data_available = False
                for s in readable:
                    if s is self.sock:
                        recv_data, address = s.recvfrom(4096)
                        if isinstance(recv_data, bytes):
                            recv_data = recv_data.decode() # recvfrom returnes bytes in python3. json.loads() receives str.
                        chunk_list.append(json.loads(recv_data))
            return chunk_list # return dict
        except BlockingIOError as e:
            print("Socket error: {}".format(e))


class MainWindow():
    '''
    Main window - GUI entry point
    '''
    def __init__(self, master, UDP_conn_gui, UDP_conn_plot):
        print("Init Plotter")
        self.plotter = Plotter()
        pandas.read_csv("obstacles_100m_above_sea_level.csv").to_numpy()
        obstacles = coordinate_convertions.obstacles_ex(pandas.read_csv("obstacles_100m_above_sea_level.csv").to_numpy())
        m_line = LineString([(config.start_pos.x, config.start_pos.y), (config.goal_pos.x, config.goal_pos.y)])
        self.plotter.plot_static(obstacles=obstacles,m_line = m_line)

        print("Init GUI")
        self.UDP_conn_gui = UDP_conn_gui # UDP connection object
        self.UDP_conn_plot = UDP_conn_plot
        self.master = master
        self.master.geometry('400x250')
        self.master.title("Drone GUI")
        self.master.bind("<q>", self.q_pressed)
        self.master.bind("<t>", self.t_pressed)

        # always on top
        self.always_on_top = tk.IntVar()
        self.always_on_top.set(1)
        self.master.wm_attributes("-topmost", 1)
        
        # Add GUI elements
        
        # Add quit button
        tk.Button(self.master, text ="Quit <q>", command = self.quit_all).grid(row=0, column=0, pady=0, sticky="W")

        # Add always on top checkbox
        tk.Checkbutton(self.master, text="Always on top <t>", variable=self.always_on_top, command=self.always_on_top_toggle).grid(row=1, column=0, pady=0, sticky="W")
        
        # Add status
        self.status_text = tk.StringVar()                 # Create new StringVar
        self.status_text.set("waiting for status update") # Update the StringVar (label's) text
        # Create the lable itself and assign a text
        self.label = tk.Label(master = self.master, textvariable = self.status_text, font = "Consolas 20", anchor = "w", justify = tk.LEFT)
        self.label.grid(row=2, column=0, sticky="W")

        self.master.after(10, self.get_new_status_msg)

    def get_new_status_msg(self):
        message = self.UDP_conn_gui.recv() # Get new message from UDP socket
        if message is not None:
            self.status_text.set(message) # Update the status

        # Get drone_pose, lidar_world_frame
        plt_msg_list = self.UDP_conn_plot.recv_list() # Get new message from UDP socket
        if plt_msg_list is not None and len(plt_msg_list) > 0:
            # Plot
            self.plotter.set_axes_limit()
            self.plotter.plot_drone_and_lidar_pos(plt_msg_list)

        self.master.after(10, self.get_new_status_msg)

    def t_pressed(self, event):
        print("'T' pressed, toggling always on top")
        self.always_on_top.set(1 - self.always_on_top.get()) # toggle the checkbox state
        self.always_on_top_toggle()
    
    def always_on_top_toggle(self):
        self.master.wm_attributes("-topmost", self.always_on_top.get()) # Always on top - Windows

    def q_pressed(self, event):
        print("'Q' pressed. Exit.")
        self.quit_all()

    def quit_all(self):
        '''
        Destructor of GUI
        '''
        self.master.destroy()


root = tk.Tk()
UDP_conn_gui = UDP(("127.0.0.1", 5077)) # UDP connection object
UDP_conn_plot = UDP(("127.0.0.1", 5078))
MainWindow(root, UDP_conn_gui, UDP_conn_plot)
root.mainloop()