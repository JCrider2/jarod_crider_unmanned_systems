# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 09:28:09 2023

@author: Jarod
"""

import numpy as np

def distance(x1:float,x2:float,y1:float,y2:float):
    out = np.sqrt((x2-x1)**2+(y1-y2)**2)
    return out


x1 = 2
x2 = 3
y1 = 1
y2 = x2

z = distance(x1,x2,y1,y2)
