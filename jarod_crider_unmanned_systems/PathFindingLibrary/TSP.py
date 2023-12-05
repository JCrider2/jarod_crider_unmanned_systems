
"""
Created on Wed Oct 25 09:48:40 2023

@author: Jarod
"""

"""
cost list = []
for loop to go through each permutation:
    set cost to 0
    for loop to each individual wp
        current cost = compute distance(wp_curr, wp_next)
    cost_list.append(cost)
"""

from itertools import permutations
import numpy as np
from AStar import AStarFun

def distance(cordA:tuple,cordB:tuple) -> float:
    dis = np.sqrt((cordB[0]-cordA[0])**2 + (cordB[1]-cordA[1])**2)
    return dis


maps = [(0,15),(0,15),0.5]
obR = 0.5

ox = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8,
      8, 8, 8, 8, 2, 3, 4, 5, 6, 9, 10, 11, 12, 15, 2, 2, 2,  2,  2,  2,  5, 5,
      5,  5,  5,  5,  5,  6,  7,  8,  9,  10, 11, 12, 12, 12, 12, 12]
oy = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2,  2,  2,  2,  3, 4, 5,
      6, 7, 8, 9, 7, 7, 7, 7, 7, 6, 6,  6,  6,  6,  8, 9, 10, 11, 12, 13, 9, 10,
      11, 12, 13, 14, 15, 12, 12, 12, 12, 12, 12, 8,  9,  10, 11, 12]

start = (0,0)
goals = [(9,4),(4,4),(9,7),(1,9),(6,14)]

perm = list(permutations(goals))

print("number of combinations is ", len(perm))

cost_list = []

for combo in perm:
    cost = 0
    previous = start
    print("test")
    for current in combo:
        curr_cost = distance(previous,current)

        cost += curr_cost
        previous = current

    
    cost_list.append(cost)
    
    
index = np.array(cost_list).argmin()
print("Lowest Cost permutation ", perm[index])
   
