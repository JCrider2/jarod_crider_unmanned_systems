# -*- coding: utf-8 -*-
"""
Created on Wed Sep 13 12:23:48 2023

@author: Jarod
"""
# RRT

import numpy as np

"""
    - node data class
        - Attributes
            - x,y,parent_cost,index
"""
class node:
    def __init__(self,x:float,y:float,parent_iter:int,cost:float):
        self.x = x
        self.y = y
        self.parent_iter = parent_iter
        self.cost = cost
        

"""
    - object data class
        - Attributes
            - x_pos,y_pos,obstacle_radius
        - Methods
            - is_inside -checks if input cords interfere with object
"""
class Obstacle():
    def __init__(self, x_pos:float, y_pos:float, radius:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
    def is_inside(self, curr_x:float, curr_y:float, robot_radius:float=0):
        """
        Compute euclidian distance from current location -> obstalc location
        stor this value: dist_from
        if dist_form > obstacle size
            return false
        otherwise return true
        """
        
        dist_from = np.sqrt((curr_x - self.x_pos)**2+ (curr_y - self.y_pos)**2)
        if dist_from > self.radius+robot_radius:
            return False
        return True

"""
Dijkstra's Algorithm Pseudo

-Function
    !-Distance
        -Inputs
            -Node1
            -Node2
        -Outputs
            - Distance from one another
        -Blackbox
            - Sqrt numpy
"""
def distance(x1:float,x2:float,y1:float,y2:float):
    out = np.sqrt((x2-x1)**2+(y1-y2)**2)
    return out

"""            
    - Collision Check
        -Inputs
            - graph size x_min, x_max, y_min, y_max, gs,
                x_curr, y_curr, robot radius
        - Outputs
            True - inbounds, no object
            False - out of bounds or object interference
"""      
def valid_check(x_min:float,x_max:float,y_min:float,y_max:float,
                  x_curr:float,y_curr:float,obst_list:list,robot_radius:float=0.0):    
    for jar in obst_list:
        if jar.is_inside(x_curr,y_curr,robot_radius):
            return False     
    if x_min <= x_curr <= x_max and y_min <= y_curr <= y_max:
        return True    # Inside barrier
    return False     # Outside barrier

def merge(x,y):
    merged_list = [(x[i], y[i]) for i in range(0, len(x))]
    return merged_list




