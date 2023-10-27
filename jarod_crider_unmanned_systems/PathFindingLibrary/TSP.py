
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

def distance(cordA:tuple,cordB:tuple) -> float:
    dis = np.sqrt((cordB[0]-cordA[0])**2 + (cordB[1]-cordA[1])**2)
    return dis


start = (0,0)
goals = [(2,2),(5,3),(3,4),(6,4)]

perm = list(permutations(goals))

print("number of combinations is ", len(perm))

cost_list = []

for combo in perm:
    cost = 0
    previous = start
    for current in combo:
        curr_cost = distance(previous,current)
        cost += curr_cost
        previous = current

    
    cost_list.append(cost)
    
    
index = np.array(cost_list).argmin()
print("Lowest Cost permutation ", perm[index])
   
