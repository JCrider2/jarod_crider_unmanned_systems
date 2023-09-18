# -*- coding: utf-8 -*-
"""
Created on Fri Sep  1 10:14:56 2023

@author: Jarod
"""

class node:
    def __init__(self,x):
        self.x = x




test = {1:node(1),3:node(-4),99:node(5),24:node(22)}

for t, y in test.items():
    print(t,y.x)