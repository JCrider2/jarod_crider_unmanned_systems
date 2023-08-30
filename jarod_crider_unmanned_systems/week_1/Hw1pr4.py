# -*- coding: utf-8 -*-
"""
Created on Wed Aug 23 10:44:50 2023

@author: Jarod
"""

import matplotlib.pyplot as plt
import numpy as np

# Compute Index based on grid size and location
def compute_index(min_x:int, max_x:int, min_y:int, max_y, gs:int, x_curr:int, y_curr:int) -> int:
    
    index = ((x_curr - min_x)/gs) + (((y_curr - min_y)/gs) * ((max_x-min_x+gs)/gs))
    
    return index


    
min_x = 0
max_x = 10
min_y = 0
max_y = 10
gs = 0.5
x_array = []
x_curr = 7
y_curr = 3.5

index = compute_index(min_x,max_x,min_y,max_y,gs,x_curr,y_curr)
print(index)
       
    
# Value input for graphing    
x_cord = list(np.arange(min_x,max_x+gs,gs))
y_cord = list(np.arange(min_y,max_y+gs,gs))

for j, y in enumerate(y_cord):
    for i, x in enumerate(x_cord):
        index = compute_index(min_x,max_x,min_y,max_y,gs,x,y)
        plt.text(x, y, str(int(index)), color="red", fontsize=64)

