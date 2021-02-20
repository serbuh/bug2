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


if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    # start = (-1170, -981,-100)
    # goal = (-1170, -522,-100)
    #down - > up
    # start = (-414, -860,-100)
    # goal = (-402, -727, -100)
    # left-> right
    start = (-375,-930,-100)
    goal = (-367,-682,-100)
    speed = 5

    time.sleep(4)
    client.setAtPosition(start[0],start[1],start[2])
    time.sleep(3)
    client.flyToPosition(goal[0], goal[1],goal[2], speed)

    plotter1 = plotter()
    
    pandas.read_csv("obstacles_100m_above_sea_level.csv").to_numpy()

    obstacles  = obstacles_ex.obstacles_ex(pandas.read_csv("obstacles_100m_above_sea_level.csv").to_numpy())
    m_line= LineString([(start[0],start[1]), (goal[0],goal[1])])
    

    plotter1.plot_static(obstacles=obstacles,m_line = m_line)

    bug2 = obstacles_ex.bug2(start, goal, speed, client)    

    while True:
        plotter1.set_axes_limit()
        
        res = client.getPose()
        drone_position = Point(res.pos.x_m, res.pos.y_m, res.pos.z_m)
        lidar_relative_drone = client.getLidarData()
        obstacles_ex.get_lidar_world_frame(lidar_relative_drone,res)
        lidar_world_frame = obstacles_ex.get_lidar_world_frame(lidar_relative_drone,res)
        if lidar_world_frame == "no obstacle detected":
            lidar_world_frame = np.array([0,0,0])
        plotter1.plot_drone_and_lidar_pos(res=res, lidar_world_frame=lidar_world_frame)

        bug2.run(lidar_relative_drone, lidar_world_frame, drone_position)
       



        
