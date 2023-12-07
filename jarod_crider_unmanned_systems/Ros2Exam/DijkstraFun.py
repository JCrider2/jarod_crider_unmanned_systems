# -*- coding: utf-8 -*-
"""
Created on Sat Sep  2 09:12:09 2023

@author: Jarod
"""
# DijkstraFunTime Redo
from unmanned_systems_ros2_pkg import DijkstraFunTime as Dft

def DijkstraFun(map:list,obstacle_x:list,obstacle_y:list,obstacle_radius:float,start:tuple,finish:tuple,robot_radius:float):
    
    obstacle_positions = Dft.merge(obstacle_x,obstacle_y)
    
    obstacle_list = [] # Store obstacle objects
    # map = [(xmin,xmax),(ymin,ymax),gs]
    xmin = map[0][0]
    xmax = map[0][1]   # Map Sizing
    ymin = map[1][0]  
    ymax = map[1][1]
    gs = map[2]
    
    x_start = start[0]
    y_start = start[1] # Start
    
    x_finish = finish[0] # Finish
    y_finish = finish[1]


    
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
    current_node = Dft.node(x_start,y_start,0.0,-1)
    index = Dft.compute_index(xmin,xmax,ymin,ymax,gs,current_node.x,current_node.y)
    
    
    # This While loop is Dijkstra's, assembles visted dictionary with object having parent index, and cost
    while unvisited:
        
        for j in range(-1,2):       # 3x3 grid check
            for i in range(-1,2):
                
                if i == 0 and j == 0: # Used to skip looking center of 3x3
                    continue
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
                                continue
                            
                        # Is the scanned node visited already
                        elif i_scan in visited:
                            continue
                        
                        # Scan node is new
                        else:
                            cost_scan = Dft.distance(x_scan,current_node.x,y_scan,current_node.y)+current_node.cost
                            node = Dft.node(x_scan,y_scan,cost_scan,index)
                            unvisited[i_scan] = node
                    else:
                        continue
        
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

    x_list.reverse()
    y_list.reverse()

    cord_list = Dft.merge(x_list,y_list)
    cord_list.reverse()
    final_cost = 0
    
    look = len(cord_list)-1
    
    for t in range(look):
        Ax = cord_list[t][0]
        Ay = cord_list[t][1]
        Bx = cord_list[t+1][0]
        By = cord_list[t+1][1]
        
        
        final_cost += Dft.distance(Ax,Bx,Ay,By)
    
    
    return cord_list
