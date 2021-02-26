from DroneClient import DroneClient
import time
import airsim.utils
#from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString 
from shapely.geometry import Point
import numpy as np
import pandas 
import matplotlib.pyplot as plt

class plotter:
    def __init__(self):
        plt.ion()
        self.fig = plt.figure()
        self.ax_obstacle = self.fig.add_subplot(111)
        self.ax_obstacle.invert_xaxis()
        self.ax_drone = self.ax_obstacle.twinx()
        self.ax_lidar= self.ax_obstacle.twinx() # relative to world axes
        

    def plot_static(self,obstacles,m_line): #plot obstacle and m-line
        for obstacle in obstacles:
            self.ax_obstacle.add_patch(plt.Polygon(obstacle.exterior.coords, color='b'))
            self.ax_obstacle.plot(list(m_line.xy[0]), list(m_line.xy[1]), color='red', linewidth=1,alpha=0.1)
            
             
    def set_axes_limit(self):
        min = -1200
        max = -350
        self.ax_obstacle.set_ylim(bottom=min,top=max,auto=False)
        self.ax_drone.set_ylim(bottom=min,top=max,auto=False)
        self.ax_lidar.set_ylim(bottom=min,top=max,auto=False)
        self.fig.tight_layout(pad=1.08, h_pad=None, w_pad=None, rect=None)

    
    def plot_drone_and_lidar_pos(self, drone_pose, lidar_world_frame):
        self.ax_lidar.plot((lidar_world_frame[0]),(lidar_world_frame[1]), 'o', color='red')
        self.ax_drone.plot((drone_pose.pos.x_m),(drone_pose.pos.y_m), 'o', color='black')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.ax_drone.clear()
    