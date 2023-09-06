# -*- coding: utf-8 -*-
"""
Created on Sat Sep  2 09:12:09 2023

@author: Jarod
"""
# DijkstraFunTime Redo
import DijkstraFunTime as Dft
from operator import attrgetter
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    
    obstacle_positions = [(1,1),(4,4),(3,4),(5,0),(5,1),(0,7),(1,7),(2,7),(3,7)]
    obstacle_list = [] # Store obstacle classes
    obstacle_radius = 0.5
    
    xmin = 0
    xmax = 10   # Map Sizing
    ymin = 0
    ymax = 10
    gs = 0.5
    
    x_start = 0.0
    y_start = 0.0 # Start
    
    x_finish = 8.0 # Finish
    y_finish = 9.0

    robot_radius = 0.5
    
    for obs_pos in obstacle_positions:
        obstacle = Dft.Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)    # Change the obstacle form
        obstacle_list.append(obstacle)

    visited = {}
    unvisited = {}  # Initialize data keeping dictionary
    
    x_list = []     #Final cord lists
    y_list = []
    
    parent_index = Dft.compute_index(xmin,xmax,ymin,ymax,gs,x_finish,y_finish)
    last_node = Dft.node(x_finish,y_finish,parent_index,0.0,parent_index)
    unvisited[parent_index] = last_node
    current_index = Dft.compute_index(xmin,xmax,ymin,ymax,gs,x_start,y_start)
    current_node = Dft.node(x_start,y_start,0.0,-1,current_index)
    
    while unvisited:
        
        index = Dft.compute_index(xmin,xmax,ymin,ymax,gs,current_node.x,current_node.y)
        
        for j in range(-1,2):       # 3x3 grid check
            for i in range(-1,2):
                
                if i == 0 and j == 0: # Used to skip looking center of 3x3
                    pass
                else:
                    x_scan = current_node.x+i*gs
                    y_scan = current_node.y+j*gs
                    i_scan = Dft.compute_index(xmin,xmax,ymin,ymax,gs,x_scan,y_scan)
                    if Dft.valid_check(xmin,xmax,ymin,ymax,x_scan,y_scan,obstacle_list,robot_radius):
                        
                        if i_scan in unvisited:
                            node_scan = unvisited[i_scan]
                            old_cost_scan = node_scan.cost
                            new_cost_scan = Dft.distance(node_scan.x,current_node.x,node_scan.y,current_node.y)+current_node.cost
                            if old_cost_scan > new_cost_scan:
                                node_scan.cost = new_cost_scan
                                node_scan.parent_index = current_node.index
                                unvisited[i_scan] = node_scan
                            else:
                                pass
                        elif i_scan in visited:
                            continue
                        else:
                            cost_scan = Dft.distance(x_scan,current_node.x,y_scan,current_node.y)+current_node.cost
                            node = Dft.node(x_scan,y_scan,cost_scan,index,i_scan)
                            unvisited[i_scan] = node
                    else:
                        pass
        visited.update({index:current_node})
        current_node = min(unvisited.values(), key=attrgetter('cost'))
        del unvisited[current_node.index]
                    
    while parent_index != -1:
        back = visited[parent_index]
        x_list.append(back.x)
        y_list.append(back.y)
        parent_index = back.parent_index

    fig, ax = plt.subplots()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)

    x_list = np.array(x_list)
    y_list = np.array(y_list)
    plt.plot(x_list,y_list)


    for obs in obstacle_list:
        obs_plot = plt.Circle((obs.x_pos, obs.y_pos), obs.radius, color="blue")
        ax.add_patch(obs_plot)
    plt.show()





