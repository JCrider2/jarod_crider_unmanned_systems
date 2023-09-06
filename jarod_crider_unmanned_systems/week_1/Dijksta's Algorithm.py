# -*- coding: utf-8 -*-
"""
Created on Wed Aug 30 18:48:21 2023

@author: Jarod
"""

import DijkstasAlgorithm as Dij


t = 0       # For testing purposes helps with the kill switch
if __name__ == "__main__":
    obstacle_positions = [(1,1),(4,4),(3,4),(5,0),(5,1),(0,7),(1,7),(2,7),(3,7)]
    obstacle_list = [] # Store obstacle classes
    obstacle_radius = 0.5
    map_size = [(0,10),(0,10)]# x_min x_max y_min, y_max
    gs = 0.5
    start = (0,0)
    finish = (8,9)
    robot_rad = 0
    
    for obs_pos in obstacle_positions:
        obstacle = Dij.Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
        obstacle_list.append(obstacle)
        
    index = int(Dij.compute_index(map_size,gs,start)       )
    current_node = Dij.node(index,(start[0],start[1]),0,0)
    index = Dij.compute_index(map_size,gs,finish)
    lat_cord = Dij.dist(start,finish)**2
    last_node = Dij.node(index,(finish[0],finish[1]),-1,lat_cord)
    visited = []
    unvisited = []
    unvisited.append(last_node)
    unvisited_store = []
    parent_cost = 0.0
    while unvisited:
        cord_cur = (0.0,0.0)
        index = 0
        old_cost = 0.0
        new_cost = 0.0
        cost = 0.0
        
        parent_cord = (current_node.cord[0],current_node.cord[1])
        for j in range(-1,2):
            for i in range(-1,2):

                
                if i == 0 and j == 0:
                    pass
                else:
                    cord_cur = (current_node.cord[0]+i*gs,current_node.cord[1]+j*gs)
                    if Dij.barrier_check(map_size,cord_cur,obstacle_list,robot_rad):    # Valid Spot?
                        index = int(Dij.compute_index(map_size,gs,cord_cur))
                        
                        
                        for k in unvisited:   # Previously looked at?
                            if index == k.dex:      # Looked at
                                old_cost = k.cost
                                new_cost = Dij.dist(parent_cord,cord_cur) + parent_cost
                                if new_cost < old_cost:
                                    k.cost = new_cost
                                else:
                                    k.cost = old_cost
                                    
                            else:                   # Not looked at
                                cost = Dij.dist(parent_cord,cord_cur) + current_node.cost
                                update_node = Dij.node(index,cord_cur,current_node.cost,cost)
                                unvisited_store.append(update_node)
        unvisited = unvisited + unvisited_store
        unvisited_store.clear()
        unvisited.sort(key=lambda p: p.cost)
        visited.append(current_node)
        current_node = unvisited.pop(0)

        """
        to tweek
        the cost and parent_cost needs updated
        
        
        """
        
        
                
                
        t = 1 + t
        if t >= 7:     #Kill Switch
            break