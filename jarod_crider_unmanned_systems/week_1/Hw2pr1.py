# -*- coding: utf-8 -*-
"""
Created on Wed Aug 30 16:27:44 2023

@author: Jarod
"""
import numpy as np

def distance(nod1x:int,nod1y:int,nod2x:int,nod2y:int) -> float:
    dist = np.sqrt((nod2x-nod1x)**2+(nod2y-nod1y)**2)
    return dist

nod1x = 2
nod1y = 1
nod2x = 3
nod2y = 2

test = distance(nod1x,nod1y,nod2x,nod2y)

