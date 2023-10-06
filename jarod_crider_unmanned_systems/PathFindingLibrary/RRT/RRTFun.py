# -*- coding: utf-8 -*-
"""
Created on Wed Sep 13 12:26:24 2023

@author: Jarod
"""
# RRTFun
"""
    -Inputs
        - Start x,y
        - Finish x,y
        - Obstacle List
        - Grid size
        - Step size
    - Outputs
        - List of path
        - Graph of path
"""

import RRTFunTime as Rrt
import random
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    
    obstacle_positions = [(2,2),(2,3),(2,4),(5,5),(5,6),(6,6),(7,3),(7,4),(7,5),(7,6),(8,6)]
    obstacle_list = []
    obstacle_radius = 0.25
    
    xmin = 0
    xmax = 10
    ymin = 0
    ymax = 10
    gs = 1
    
    x_start = 1
    y_start = 1
    
    x_fin = 9
    y_fin = 8
    
    robot_radius = 0.5
    step_size = 0.5
    
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
        if i_new >= 1000:
            break
        
        
        
    while i_new != -1:
        back = node[i_new]
        x_list.append(back.x)
        y_list.append(back.y)
        i_new = back.parent_iter
    
    # Plotting/Graphing
    fig, ax = plt.subplots()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    x_list = np.array(x_list)
    y_list = np.array(y_list)
    plt.plot(x_list,y_list)
    
    for value in node.values():
        plt.figure()
        x1 = value.x
        y1 = value.y
        pi = value.parent_iter
        if pi == -1:
            continue
        parent_node = node[pi]
        x2 = parent_node.x
        y2 = parent_node.y
        plt.plot(x1,y1,x2,y2)
        
    for obs in obstacle_list:
        obs_plot = plt.Circle((obs.x_pos, obs.y_pos), obs.radius, color="blue")
        ax.add_patch(obs_plot)
    plt.grid()
    plt.show()
    

    
    
    
    print(cost_curr+2*step_size)
    
    