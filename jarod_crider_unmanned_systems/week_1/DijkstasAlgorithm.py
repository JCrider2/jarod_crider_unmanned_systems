# -*- coding: utf-8 -*-
"""
Created on Wed Aug 30 16:44:16 2023

@author: Jarod
"""
import numpy as np
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
    - Collision Check
        -Inputs
            -Obstacle list
            - Grid Bounds
            - Current Node
        - Outputs
            True / False
    - Calculate node Index
        - Inputs
            -Grid bounds
            -Grids spacing
            - Chosen node input
        - Output
            - Node 
        - BlackBox
            - Hw1pr1 Compute Index
    - Node Neighbor Cost
        - Inputs
            - Current Node x , y
            - gs
        - Outputs
            - Distance from node origin to other nodes,
        - BlackBox
            - Iterate through all neighboring nodes
            - Collision check
    -Dijkstra's Algorithm
        - Inputs
            - Starting Node
            - Ending Node
            - Obstacle List
            - Map Boundary
        -Outputs
            - Chart/Graph/Plot results
            - Overall cost/distance from starting done to ending node
        - BlackBox
            - Each node chosen has a parent node 
            - A will have iterative cost added, and the preivous node to parent node

"""
# Compute node to node distance

def dist(curr:tuple,search:tuple):
    dis = np.sqrt((curr[0]-search[0])**2+(curr[1]-search[1])**2)
    return dis


# Compute Index value
def compute_index(bound:float, gs:float, current:float) -> int:
    
    x_min = bound[0][0]
    x_max = bound[0][1]
    y_min = bound[1][0]
    y_max = bound[1][1]
    
    x_curr = current[0]
    y_curr = current[1]
    
    index = ((x_curr - x_min)/gs) + (((y_curr - y_min)/gs) * ((x_max-x_min+gs)/gs))
    
    return index

# Used for Obstacles
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

class node():
    def __init__(self, dex:int, cord:tuple, parent_cost:float, cost:float):
        self.cord = cord
        self.parent_cost = parent_cost
        self.cost = cost
        self.dex = dex



def barrier_check(bound:list,current:tuple,obst_list:list,robot_radius:float=0.0):
    x_min = bound[0][0]
    x_max = bound[0][1]
    y_min = bound[1][0]
    y_max = bound[1][1]
    
    x_curr = current[0]
    y_curr = current[1]
    
    for obs in obst_list:
        if obs.is_inside(current[0],current[1],robot_radius):
            return False    
    
    if x_min <= x_curr <= x_max and y_min <= y_curr <= y_max:
        return True    # Inside barrier
    return False     # Outside barrier
