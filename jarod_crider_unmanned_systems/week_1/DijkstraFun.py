# -*- coding: utf-8 -*-
"""
Created on Sat Sep  2 09:12:09 2023

@author: Jarod
"""
# DijkstraFunTime Redo
import DijkstraFunTime as Dft
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    
    obstacle_positions = [(1,1),(4,4),(3,4),(5,0),(5,1),(0,7),(1,7),(2,7),(3,7)]
    obstacle_list = [] # Store obstacle objects
    obstacle_radius = 0.25
    
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
    
    # Change obstalce's from list of tuples into list of objects
    for obs_pos in obstacle_positions:
        obstacle = Dft.Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)    
        obstacle_list.append(obstacle)

    visited = {}
    unvisited = {}  # Initialize data keeping dictionary
    
    x_list = []     #Final cord lists
    y_list = []
    
    # Initialize data used in Dijkstra's
    parent_index = Dft.compute_index(xmin,xmax,ymin,ymax,gs,x_finish,y_finish)
    last_node = Dft.node(x_finish,y_finish,parent_index,0.0)
    unvisited[parent_index] = last_node
    current_index = Dft.compute_index(xmin,xmax,ymin,ymax,gs,x_start,y_start)
    current_node = Dft.node(x_start,y_start,0.0,-1)
    
    while unvisited:
        
        index = Dft.compute_index(xmin,xmax,ymin,ymax,gs,current_node.x,current_node.y)
        
        for j in range(-1,2):       # 3x3 grid check
            for i in range(-1,2):
                
                if i == 0 and j == 0: # Used to skip looking center of 3x3
                    pass
                else:
                    # get that scaned x,y,index values
                    x_scan = current_node.x+i*gs
                    y_scan = current_node.y+j*gs
                    i_scan = Dft.compute_index(xmin,xmax,ymin,ymax,gs,x_scan,y_scan)
                    if Dft.valid_check(xmin,xmax,ymin,ymax,x_scan,y_scan,obstacle_list,robot_radius):
                        
                        # Was the scan node already scanned
                        if i_scan in unvisited:
                            node_scan = unvisited[i_scan]
                            old_cost_scan = node_scan.cost
                            new_cost_scan = Dft.distance(node_scan.x,current_node.x,node_scan.y,current_node.y)+current_node.cost
                            
                            # Comparing new calculated cost with old calculated cost
                            if old_cost_scan > new_cost_scan:
                                node_scan.cost = new_cost_scan
                                node_scan.parent_index = index
                                unvisited[i_scan] = node_scan
                            else:
                                pass
                            
                        # Is the scanned node visited already
                        elif i_scan in visited:
                            continue
                        
                        # Scan node is new
                        else:
                            cost_scan = Dft.distance(x_scan,current_node.x,y_scan,current_node.y)+current_node.cost
                            node = Dft.node(x_scan,y_scan,cost_scan,index)
                            unvisited[i_scan] = node
                    else:
                        pass
        
        # Move current node to visited, find lowest cost select that, and delete
        visited.update({index:current_node})
        index = min(unvisited, key=lambda x:unvisited[x].cost)
        current_node = unvisited.pop(index)
                    
    # Getting cordinates from finish to start    
    while parent_index != -1:
        back = visited[parent_index]
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


    for obs in obstacle_list:
        obs_plot = plt.Circle((obs.x_pos, obs.y_pos), obs.radius, color="blue")
        ax.add_patch(obs_plot)
    plt.grid()
    plt.show()





