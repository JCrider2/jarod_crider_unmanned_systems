# -*- coding: utf-8 -*-
"""
Created on Mon Oct 23 22:33:39 2023

@author: Jarod
"""

from unmanned_systems_ros2_pkg import RRTFunTime as Rrt
import random
import numpy as np

def RRT(graph:list,obs_x:list,obs_y:list,obj_radi:float,
        start:tuple,finish:tuple,robot_radius:float,step_size:float):

    obstacle_positions = Rrt.merge(obs_x,obs_y)
    obstacle_list = []
    obstacle_radius = obj_radi
        
    xmin = graph[0][0]
    xmax = graph[0][1]
    ymin = graph[1][0]
    ymax = graph[1][1]
     
    x_start = start[0]
    y_start = start[1]
      
    x_fin = finish[0]
    y_fin = finish[1]
        
    x_list = []
    y_list = []
        
    for obs_pos in obstacle_positions:
        obstacle = Rrt.Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)    
        obstacle_list.append(obstacle)
                
    current_node = Rrt.node(x_start,y_start,-1,0)
    node = {0:current_node}
        
    i_new = 0
    while 1 == 1:
        x_guess = round(random.uniform(xmin,xmax),1)
        y_guess = round(random.uniform(ymin,ymax),1)
        dist_curr = float('inf')
            
            # Iterate through nodes for closes to node to random point
        for key, letter in node.items():
            x_test = letter.x
            y_test = letter.y
            dist = Rrt.distance(x_guess, x_test, y_guess, y_test)
            if dist < dist_curr:
                dist_curr = dist
                x_curr = x_test
                y_curr = y_test
                i_curr = key
                cost_curr = letter.cost
            
            # Math for step
        theta = np.arctan2(x_guess-x_curr,y_guess-y_curr)
        x_new = step_size * np.sin(theta) + x_curr
        y_new = step_size * np.cos(theta) + y_curr
            
            # Valid Check, if inside border, outside obstacle
        if Rrt.valid_check(xmin,xmax,ymin,ymax,x_new,y_new,obstacle_list,robot_radius):
            i_new += 1
            new_node = Rrt.node(x_new,y_new,i_curr,cost_curr+step_size)
            node[i_new] = new_node
            
            # Check if new node is close to ending spot
        final_distance = Rrt.distance(x_fin,x_new,y_fin,y_new)
        if final_distance <= step_size:
            i_new += 1
            node[i_new] = Rrt.node(x_fin,y_fin,i_new-1,cost_curr+2*step_size)
             
            break
                
    while i_new != -1:
        back = node[i_new]
        x_list.append(back.x)
        y_list.append(back.y)
        i_new = back.parent_iter
    
    # Plotting/Graphing
    x_list.reverse()
    y_list.reverse()

    lust = Rrt.merge(x_list,y_list)
    
    return lust
    


