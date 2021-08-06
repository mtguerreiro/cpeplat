# -*- coding: utf-8 -*-
"""
Created on Thu Aug  5 09:16:57 2021

@author: Moritz Frantz
"""

import cpeplat as cpe
import numpy as np
import matplotlib.pyplot as plt
import time
import cpeplat.result as res

##########-------------- Basic Setting Functions  ----------------############

def _setting_labels(type_of):
    if type_of == 'Voltage':
        plt.xlabel("Time / ms", fontsize=12)
        plt.ylabel("Voltage / V", fontsize=12)  
    
    elif type_of == 'Current':
        plt.xlabel("Time / ms", fontsize=12)
        plt.ylabel("Current / A", fontsize=12) 
        
    elif type_of == 'ControlSignal':
        plt.xlabel("Time / ms", fontsize=12)
        plt.ylabel("u (DutyCycle)", fontsize=12)        
        
def _setting_legend(leg):
        plt.gca().legend(leg, fontsize=12)
        
def _setting_title(title):
    plt.title(title, fontsize=12)



##########-------------- Basic Plot Functions  -------------------############

def plot_vin(data):
        
    #Expected DataType np.array
    plt.figure(figsize=(9, 6))
    plt.plot(data[0],data[1])
        
    res._setting_labels('Voltage')
    res._setting_title('Input Voltage')
    plt.grid()
    return
    
def plot_vin_buck(data):

    #Expected DataType np.array
    plt.figure(figsize=(9, 6))
    plt.plot(data[0],data[2])
        
    res._setting_labels('Voltage')
    res._setting_title('Input Voltage Buck')
    plt.grid()
    return       

def plot_vout(data):
        
    #Expected DataType np.array
    plt.figure(figsize=(9, 6))
    plt.plot(data[0],data[3])
        
    res._setting_labels('Voltage')
    res._setting_title('Output Voltage')
    plt.grid()
    return

def plot_vout_buck(data):
        
    #Expected DataType np.array
    plt.figure(figsize=(9, 6))
    plt.plot(data[0],data[4])
        
    res._setting_labels('Voltage')
    res._setting_title('Output Voltage Buck')
    plt.grid()
    return

def plot_il(data):
        
    #Expected DataType np.array
    plt.figure(figsize=(9, 6))
    plt.plot(data[0],data[5])
        
    res._setting_labels('Current')
    res._setting_title('Output Current')
    plt.grid()
    return

def plot_il_avg(data):
        
    #Expected DataType np.array
    plt.figure(figsize=(9, 6))
    plt.plot(data[0],data[6])
        
    res._setting_labels('Current')
    res._setting_title('Output Current Average')
    plt.grid()
    return

def plot_u(data):
        
    #Expected DataType np.array
    plt.figure(figsize=(9, 6))
    plt.plot(data[0],data[7])
        
    res._setting_labels('ControlSignal')
    res._setting_title('Control Signal u')
    plt.grid()
    return

##########-------------- Comparision Plot Functions  -------------############

def plot_compare_vin(data, leg = ['Controler_1','Controler_2']):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    
    for i in data:
        plt.plot(i[0],i[1])
        
    res._setting_labels('Voltage')
    res._setting_title('Input Voltage')
    plt.gca().legend(leg)
    plt.grid()

def plot_compare_vin_buck(data, leg = ['Controler_1','Controler_2']):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    
    for i in data:
        plt.plot(i[0],i[2])
        
    res._setting_labels('Voltage')
    res._setting_title('Input Voltage Buck')
    plt.gca().legend(leg)
    plt.grid()
    
def plot_compare_vout(data, leg = ['Controler_1','Controler_2']):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    
    for i in data:
        plt.plot(i[0],i[3])
        
    res._setting_labels('Voltage')
    res._setting_title('Output Voltage')
    plt.gca().legend(leg)
    plt.grid()

    
def plot_compare_vout_buck(data, leg = ['Controler_1','Controler_2']):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    
    for i in data:
        plt.plot(i[0],i[4])
        
    res._setting_labels('Voltage')
    res._setting_title('Output Voltage Buck')
    plt.grid()
    
def plot_compare_il(data, leg = ['Controler_1','Controler_2']):
    """ Compares Data from experiments and plots it in one Figure.
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    
    for i in data:
        plt.plot(i[0],i[5])
        
    res._setting_labels('Current')
    res._setting_title('Output Current')
    plt.gca().legend(leg)
    plt.grid()
    
def plot_compare_il_avg(data, leg = ['Controler_1','Controler_2']):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2])
    """
    
    plt.figure(figsize=(9, 6))
    
    for i in data:
        plt.plot(i[0],i[6])
        
    res._setting_labels('Current')
    res._setting_title('Output Current Average')
    plt.gca().legend(leg)
    plt.grid()
    
def plot_compare_u(data, leg = ['Controler_1','Controler_2']):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    
    for i in data:
        plt.plot(i[0],i[7])
        
    res._setting_labels('ControlSignal')
    plt.gca().legend(leg)
    plt.grid()   
    
def plot_compare_all(data, leg = ['Controler_1','Controler_2']):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(16, 9))
    plt.subplot(1, 3, 1)
    for i in data:
        plt.plot(i[0],i[3])
        
    res._setting_labels('Voltage')
    res._setting_title('Output Voltage')
    plt.gca().legend(leg)
    plt.grid()   
    
    plt.subplot(1, 3, 2)
    for i in data:
        plt.plot(i[0],i[6])
        
    res._setting_labels('Current')
    res._setting_title('Average Current')
    plt.gca().legend(leg)
    plt.grid() 
    
    plt.subplot(1, 3, 3)
    for i in data:
        plt.plot(i[0],i[7])
        
    res._setting_labels('ControlSignal')
    res._setting_title('Control Signal u')
    plt.gca().legend(leg)
    plt.grid()