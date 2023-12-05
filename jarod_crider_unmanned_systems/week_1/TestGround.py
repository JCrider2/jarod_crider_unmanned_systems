# -*- coding: utf-8 -*-
"""
Created on Fri Sep  1 10:14:56 2023

@author: Jarod
"""

from numba import jit
import numpy as np

from timeit import default_timer as timer


def func(a):
    for i in range(10000000):
        a[i]+= 1
        
@jit(target_backend='cuda')
def func2(a):
    for i in range(10000000):
        a[i]+= 1
        
if __name__=="__main__":
    n = 10000000
    a = np.ones(n, dtype = np.float64)
    
    start = timer()
    func(a)
    print("without GPU:", timer()-start)
    
    start = timer()
    func2(a)
    print("with GPU", timer()-start)