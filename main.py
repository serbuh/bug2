from DroneClient import DroneClient
import time
import airsim.utils
from shapely.geometry.polygon import Polygon, LineString 
from shapely.geometry import Point
import numpy as np
import pandas 
import matplotlib.pyplot as plt
import obstacles_ex
from plotter2 import plotter
from config import Config
import socket
import json
import math

if __name__ == "__main__":
    config = Config() # load config
    print("Init the simulator...")
    client = DroneClient()
    print("Connecting to the simulator...")
    client.connect()

    print("Connected to simulator: {}".format(client.isConnected()))

    time.sleep(4)
    client.setAtPosition(config.start_pos[0], config.start_pos[1], config.start_pos[2])

    #plotter1 = plotter()
    
    pandas.read_csv("obstacles_100m_above_sea_level.csv").to_numpy()

    obstacles  = obstacles_ex.obstacles_ex(pandas.read_csv("obstacles_100m_above_sea_level.csv").to_numpy())
    m_line= LineString([(config.start_pos[0], config.start_pos[1]), (config.goal_pos[0], config.goal_pos[1])])
    

    #plotter1.plot_static(obstacles=obstacles,m_line = m_line)
    print("Init bug2")
    bug2 = obstacles_ex.bug2(config, client)    
    print("Lets fly!")
    while True:
        #plotter1.set_axes_limit()
        
        drone_pose = client.getPose()
        drone_position = Point(drone_pose.pos.x_m, drone_pose.pos.y_m, drone_pose.pos.z_m)
        drone_azimuth = math.degrees(drone_pose.orientation.z_rad)
        lidar_relative_drone = client.getLidarData()
        lidar_world_frame = obstacles_ex.get_lidar_world_frame(lidar_relative_drone, drone_pose)
        if lidar_world_frame is None:
            lidar_world_frame = np.array([0,0,0])
        #plotter1.plot_drone_and_lidar_pos(drone_pose=drone_pose, lidar_world_frame=lidar_world_frame)

        bug2.run_iteration(lidar_relative_drone, lidar_world_frame, drone_position, drone_azimuth)
       



        
