# -*- coding: utf-8 -*-
"""
Created on Wed Aug 30 16:44:16 2023

@author: Jarod
"""

"""
Dijkstra's Algorithm Pseudo

-Function
    -Distance
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
    - Extra stuff
        - yet_visited, visited, no_visit.
        - Introduce robot radius size

"""