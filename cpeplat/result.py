# -*- coding: utf-8 -*-
"""
Created on Thu Aug  5 09:16:57 2021

@author: Moritz Frantz
"""

import cpeplat as cpe
import numpy as np
import matplotlib.pyplot as plt
import time


def result_plotVout(data):
        
    #Expected DataType np.array
    plt.figure(figsize=(4, 3))
    plt.plot(data[0],data[3])
        
    plt.xlabel("Time / ms")
    plt.ylabel("Voltage / V")
    #add grid
    return
    
        