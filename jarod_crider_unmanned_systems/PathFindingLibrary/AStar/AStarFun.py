# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 12:44:18 2023

@author: Jarod
"""

from AStar import AStarFunTime as Asf
import numpy as np




def AStarFun(map:list,obs_x:list,obs_y:list,obstacle_radius:float,start:tuple,finish:tuple,robot_radius:float):
    
    # map list [(xmin,xmax),(ymin,ymax),gs]
    # obstacle list = [(x,y),(x,y),(x,y)]
    # start = (x,y)
    # finish = (x,y)

    obstacle_positions = Asf.merge(obs_x,obs_y)
    
    obstacle_list = [] # Store obstacle objects
    
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
        obstacle = Asf.Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)    
        obstacle_list.append(obstacle)

    visited = {}
    unvisited = {}  # Initialize data keeping dictionary
    
    x_list = []     #Final cord lists
    y_list = []
    
    # Initialize data used in Dijkstra's
    
    heur_scan = Asf.distance(x_start,x_finish,y_start,y_finish)
    parent_index = Asf.compute_index(xmin,xmax,ymin,ymax,gs,x_finish,y_finish)
    last_node = Asf.node(x_finish,y_finish,heur_scan,parent_index,heur_scan**2)
    unvisited[parent_index] = last_node

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

                    
    # Getting cordinates from finish to start    
    while parent_index != -1:
        back = visited[parent_index]

        x_list.append(back.x)
        y_list.append(back.y)
        
        parent_index = back.parent_index


    cord_list = Asf.merge(x_list,y_list)
    final_cost = 0
    
    look = len(cord_list)-1
    
    for t in range(look):
        Ax = cord_list[t][0]
        Ay = cord_list[t][1]
        Bx = cord_list[t+1][0]
        By = cord_list[t+1][1]
        
        
        final_cost += Asf.distance(Ax,Bx,Ay,By) 
    
    cord_list.reverse()
    

    return (final_cost, cord_list)