# -*- coding: utf-8 -*-
"""
Created on Wed Oct 25 15:08:11 2023

@author: Jarod
"""

from RRT import RRTFun2 as RRT
import time

st = time.process_time()

Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8,
              8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
              5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]

Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7,
              8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
              14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]

start = (1,1)
finish = (7,13)

map = [(0,15),(0,15)]

Obs_r = 0.25

RoboR = 0.5

Step_Size = 0.5


stuff = RRT.RRT(map,Obstacle_x,Obstacle_y,Obs_r,start,finish,RoboR,Step_Size)

path = stuff[0]
final_cost = stuff[1]



et = time.process_time()

res = et-st
