# -*- coding: utf-8 -*-
"""
Created on Mon Dec  4 18:51:52 2023

@author: Jarod
"""
from itertools import permutations
from AStar import AStarFun
import numpy as np
import matplotlib.pyplot as plt

maps = [(0,15),(0,15),0.5]


start = (0,0)
points = [(9,1),(4,4),(1,9),(9,7),(6,14)]
rad = 0.5
obr = 0.0

ox = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8,
      8, 8, 8, 8, 2, 3, 4, 5, 6, 9, 10, 11, 12, 15, 2, 2, 2,  2,  2,  2,  5, 5,
      5,  5,  5,  5,  5,  6,  7,  8,  9,  10, 11, 12, 12, 12, 12, 12]
oy = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2,  2,  2,  2,  3, 4, 5,
      6, 7, 8, 9, 7, 7, 7, 7, 7, 6, 6,  6,  6,  6,  8, 9, 10, 11, 12, 13, 9, 10,
      11, 12, 13, 14, 15, 12, 12, 12, 12, 12, 12, 8,  9,  10, 11, 12]



begin = {}

# this creates array from start to points
for i,j in enumerate(points):
    inner = AStarFun.AStarFun(maps,ox,oy,obr,start,j,rad)
    begin.update({i:inner})
    

table = {}
# this creates matrix for point to point lookup
for k,l in enumerate(points):
    hidden = {}
    for m,n in enumerate(points):
        if k == m:
            continue
        
        outer = AStarFun.AStarFun(maps,ox,oy,obr,l,n,rad)
        hidden.update({m:outer})
        
    table.update({k:hidden})

point_list = list(range(0,len(points)))
perm = list(permutations(point_list))

cost_list = []
for p in perm:
    cost = 0
    for v in range(len(p)-1):
        cost += table[p[v]][p[v+1]][0]


    cost += begin[p[0]][0]
    cost_list.append(cost)
    
index = np.array(cost_list).argmin()

minperm = perm[index]

cord_list = begin[minperm[0]][1]


for u in range(len(minperm)-1):
    cord = table[minperm[u]][minperm[u+1]][1]
    del cord[0]
    cord_list += cord



x_list = np.array(list(map(lambda x: x[0], cord_list)))
y_list = np.array(list(map(lambda y: y[1], cord_list)))


plt.plot(x_list,y_list)
plt.plot(ox,oy,'o')
plt.show()

