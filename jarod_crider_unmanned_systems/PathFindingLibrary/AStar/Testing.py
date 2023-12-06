# -*- coding: utf-8 -*-
"""
Created on Tue Dec  5 22:34:31 2023

@author: Jarod
"""

import AStarFunTime as Asf
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    
    obstacle_positions = [(2, 2), (2, 3), (2, 4), (2, 5), (0, 5), (1, 5), (2, 5), (3, 5), (4, 5), (5, 5), (5, 2), (5, 3), (5, 4), (5, 5), (8, 2), (9, 2), (10, 2), (11, 2), (12, 2), (13, 2), (8, 3), (8, 4), (8, 5), (8, 6), (8, 7), (8, 8), (8, 9), (2, 7), (3, 7), (4, 7), (5, 7), (6, 7), (9, 6), (10, 6), (11, 6), (12, 6), (15, 6), (2, 8), (2, 9), (2, 10), (2, 11), (2, 12), (2, 13), (5, 9), (5, 10), (5, 11), (5, 12), (5, 13), (6, 12), (7, 12), (8, 12), (9, 12), (10, 12), (11, 12), (12, 8), (12, 9), (12, 10), (12, 11), (12, 12)]
    obstacle_list = [] # Store obstacle objects
    obstacle_radius = 0.0
    
    xmin = 0
    xmax = 15   # Map Sizing
    ymin = 0
    ymax = 15
    gs = 0.5
    
    x_start = 9
    y_start = 4 # Start
    
    x_finish = 9.0 # Finish
    y_finish = 7.0

    robot_radius = 0.5
    
    # Change obstalce's from list of tuples into list of objects
    for obs_pos in obstacle_positions:
        obstacle = Asf.Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)    
        obstacle_list.append(obstacle)

    visited = {}
    unvisited = {}  # Initialize data keeping dictionary
    
    x_list = []     #Final cord lists
    y_list = []
    
    # Initialize data used in Dijkstra's
    
    heur_scan = Asf.distance(x_start,x_finish,y_start,y_finish)
    parent_index = Asf.compute_index(xmin,xmax,ymin,ymax,gs,x_finish,y_finish)
    last_node = Asf.node(x_finish,y_finish,heur_scan,parent_index,heur_scan**3)
    unvisited[parent_index] = last_node
    current_index = Asf.compute_index(xmin,xmax,ymin,ymax,gs,x_start,y_start)
    current_node = Asf.node(x_start,y_start,0,-1,heur_scan)
    index = Asf.compute_index(xmin,xmax,ymin,ymax,gs,current_node.x,current_node.y)
    
    
    # This While loop is Dijkstra's, assembles visted dictionary with object having parent index, and cost
    while unvisited:
        
        for j in range(-1,2):       # 3x3 grid check
            for i in range(-1,2):
                
                if i == 0 and j == 0: # Used to skip looking center of 3x3
                    continue
                else:
                    # get that scaned x,y,index values, heuretic_cost
                    x_scan = current_node.x+i*gs
                    y_scan = current_node.y+j*gs
                    i_scan = Asf.compute_index(xmin,xmax,ymin,ymax,gs,x_scan,y_scan)
                    heur_scan = Asf.distance(x_scan,x_finish,y_scan,y_finish)
                    if Asf.valid_check(xmin,xmax,ymin,ymax,x_scan,y_scan,obstacle_list,robot_radius):
                        
                        # Was the scan node already scanned
                        if i_scan in unvisited:
                            node_scan = unvisited[i_scan]
                            old_heur_scan = node_scan.heuretic_cost
                            new_heur_scan = Asf.distance(node_scan.x,current_node.x,node_scan.y,current_node.y)+current_node.cost+heur_scan
                            
                            # Comparing new calculated cost with old calculated cost
                            if old_heur_scan > new_heur_scan:
                                node_scan.cost = Asf.distance(node_scan.x,current_node.x,node_scan.y,current_node.y) + current_node.cost
                                node_scan.parent_index = index
                                node_scan.heuretic_cost = new_heur_scan
                                unvisited[i_scan] = node_scan
                            else:
                                continue
                            
                        # Is the scanned node visited already
                        elif i_scan in visited:
                            continue
                        
                        # Scan node is new
                        else:
                            cost_scan = Asf.distance(x_scan,current_node.x,y_scan,current_node.y)+current_node.cost
                            heur_plus_scan = cost_scan + heur_scan
                            node = Asf.node(x_scan,y_scan,cost_scan,index,heur_plus_scan)
                            unvisited[i_scan] = node
        
        # Move current node to visited, find lowest cost select that, and delete
        visited.update({index:current_node})
        index = min(unvisited, key=lambda x:unvisited[x].heuretic_cost)
        current_node = unvisited.pop(index)
        if parent_index in visited.keys():
            break
    path = []                
    # Getting cordinates from finish to start
    dot = Asf.compute_index(xmin,xmax,ymin,ymax,gs,x_start,y_start)
    while parent_index != dot:
        back = visited[parent_index]
        path.append(back)
        x_list.append(back.x)
        y_list.append(back.y)
        parent_index = back.parent_index


    # Plotting/Graphing
    fig, ax = plt.subplots()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)

    x_list = np.array(x_list)
    y_list = np.array(y_list)
    plt.plot(x_list,y_list)


    for rob in path:
        rob_plot = plt.Circle((rob.x,rob.y), robot_radius, color="red")
        ax.add_patch(rob_plot)

    for obs in obstacle_list:
        obs_plot = plt.Circle((obs.x_pos, obs.y_pos), obs.radius, color="blue")
        ax.add_patch(obs_plot)
        
    plt.grid()
    plt.show()