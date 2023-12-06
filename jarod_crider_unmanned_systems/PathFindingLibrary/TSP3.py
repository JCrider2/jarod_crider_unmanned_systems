# -*- coding: utf-8 -*-
"""
Created on Mon Dec  4 18:51:52 2023

@author: Jarod
"""
from itertools import permutations
from AStar import AStarFun
import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer as timer

star = timer()

maps = [(0,15),(0,15),0.5]


start = (1,1)

#points = [(9,7),(1,9),(4,4),(9,4)]
#points = [(9,7),(1,9),(4,4),(9,4),(6,14),(3,11),(14,1)]
points = [(9,7),(1,9),(4,4),(9,4),(6,14),(3,11),(14,1),(1,14),(14,14),(7,10)]


rad = 0.5
obr = 0.0

ox = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8, 8, 8, 8, 2, 3, 4, 5, 6, 9, 10, 11, 12, 15, 2, 2, 2,  2,  2,  2,  5, 5,  5,  5,  5,  6,  7,  8,  9,  10, 11, 12, 12, 12, 12, 12]
oy = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2,  2,  2,  2,  3, 4, 5, 6, 7, 8, 9, 7, 7, 7, 7, 7, 6, 6,  6,  6,  6,  8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13, 12, 12, 12, 12, 12, 12, 8,  9,  10, 11, 12]



begin = {}
startt = timer()
# this creates array from start to points
for i,j in enumerate(points):
    inner = AStarFun.AStarFun(maps,ox,oy,obr,start,j,rad,True)
    begin.update({i:inner})
    

table = {}
# this creates matrix for point to point lookup
for k,l in enumerate(points):
    hidden = {}
    for m,n in enumerate(points):
        if k == m:
            continue
        
        outer = AStarFun.AStarFun(maps,ox,oy,obr,l,n,rad,True)
        hidden.update({m:outer})
        
    table.update({k:hidden})

Astar = timer()-startt
mid = timer()
# this creates all the permutations as a list compliled as index values instead of cords
point_list = list(range(0,len(points)))
perm = list(permutations(point_list))



# this is used to add all costs together for each permutation, this includes cost of start to first point
cost_list = []
for p in perm:
    cost = 0
    for v in range(len(p)-1):
        cost += table[p[v]][p[v+1]][0]


    cost += begin[p[0]][0]
    cost_list.append(cost)
    
    
    
print("Astar time", Astar)    
print('cost config',timer() - mid)    
end = timer()
# this sets the permutation of lowest cost
index = np.array(cost_list).argmin()

minperm = perm[index]



cord_list = begin[minperm[0]][1]

# this compiles cord list together from found lowest cost perm
for u in range(len(minperm)-1):
    cord = table[minperm[u]][minperm[u+1]][1]
    del cord[0]
    cord_list += cord

print('ending', timer()-end)
# this is for ploting how to rule the world
x_list = np.array(list(map(lambda x: x[0], cord_list)))
y_list = np.array(list(map(lambda y: y[1], cord_list)))

des_x = np.array([1,9,1,4,9,6,3,14,1,14,7])
des_y = np.array([1,7,9,4,4,14,11,1,14,14,10])

plt.plot(x_list,y_list)
plt.plot(ox,oy,'o')
plt.plot(des_x,des_y,'x')
plt.show()
print("total time to calculate", timer() - star)
