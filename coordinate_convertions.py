import numpy as np
from shapely.geometry.polygon import Polygon, LineString

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