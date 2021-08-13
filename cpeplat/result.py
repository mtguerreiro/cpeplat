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
from itertools import zip_longest
import csv

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
        
def _setting_legend(leg, counter):
    i = 0
    legend = []
    
    if leg is None:
        for i in range(0,counter):
            legend.append('Controller_' + str(i+1))
        leg = legend   
    
    if len(leg) < counter:
        print('Warning: More data sets than given legend-set. Legend was extended.\nCheck Function input!')
        for i in range(len(leg),counter):
            leg.append('Controller_' + str(len(leg)+1))
        
        
    plt.gca().legend(leg, fontsize=12)
        
def _setting_title(title):
    plt.title(title, fontsize=14)



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

def plot_compare_vin(data, leg = None):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[1])
        counter_data = counter_data + 1
        
    res._setting_labels('Voltage')
    res._setting_title('Input Voltage')
    res._setting_legend(leg, counter_data)
    plt.grid()

def plot_compare_vin_buck(data, leg = None):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[2])
        counter_data = counter_data + 1
        
    res._setting_labels('Voltage')
    res._setting_title('Input Voltage Buck')
    res._setting_legend(leg, counter_data)
    plt.grid()
    
def plot_compare_vout(data, leg = None):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[3])
        counter_data = counter_data + 1
        
    res._setting_labels('Voltage')
    res._setting_title('Output Voltage')
    res._setting_legend(leg, counter_data)
    plt.grid()

    
def plot_compare_vout_buck(data, leg = None):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[4])
        counter_data = counter_data + 1
        
    res._setting_labels('Voltage')
    res._setting_title('Output Voltage Buck')
    res._setting_legend(leg, counter_data)    
    plt.grid()
    
def plot_compare_il(data, leg = None):
    """ Compares Data from experiments and plots it in one Figure.
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(9, 6))
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[5])
        counter_data = counter_data + 1
        
    res._setting_labels('Current')
    res._setting_title('Output Current')
    res._setting_legend(leg, counter_data)
    plt.grid()
    
def plot_compare_il_avg(data, leg = None):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2])
    """
    
    plt.figure(figsize=(9, 6))
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[6])
        counter_data = counter_data + 1
        
    res._setting_labels('Current')
    res._setting_title('Output Current Average')
    res._setting_legend(leg, counter_data)
    plt.grid()
    
def plot_compare_u(data, leg = None):
    """ Compares Data from experiments and plots it in one Figure
    
    Important: always give data in list! also when only 1 data set is used!
    Example: plot_compare([data1], ['pid'])
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    Expect: numpy Matrix
    """
    if len(data) == 1:
        print('yes')
        
    plt.figure(figsize=(9, 6))
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[7])
        counter_data = counter_data + 1
        
    
    res._setting_labels('ControlSignal')
    res._setting_title('Control Signal (DutyCycle')
    res._setting_legend(leg, counter_data)
    plt.grid()   
    
def plot_compare_all(data, leg = None, title = None):
    """ Compares Data from experiments and plots it in one Figure
    
    Expect: numpy Matrix
    Example: plot_compare([data1, data2],['pid_1', 'pid_2'])
    """
    
    plt.figure(figsize=(16, 9))
    plt.suptitle(title, fontsize = 16,)
    plt.subplot(1, 3, 1)
    
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[3])
        counter_data = counter_data + 1
        
    res._setting_labels('Voltage')
    res._setting_title('Output Voltage')
    res._setting_legend(leg, counter_data)
    plt.grid()   
    
    plt.subplot(1, 3, 2)
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[6])
        counter_data = counter_data + 1
        
    res._setting_labels('CurrSent')
    res._setting_title('Average Current')
    res._setting_legend(leg, counter_data)
    plt.grid() 
    
    plt.subplot(1, 3, 3)
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[7])
        counter_data = counter_data + 1
        
    res._setting_labels('ControlSignal')
    res._setting_title('Control Signal u')
    res._setting_legend(leg, counter_data)
    plt.grid()
    
    
def save_variable_csv(data_to_save, path = None, file_name = None):
    """ Saves Data given as an input in a csv file.
    
    Expect: List, Numpyarray
    file name: test.csv (Importand needs .csv)
    Example: save_variable_csv(data_to_save, path =, test.csv = None)
    """       
    if path is None:
        # use this for setting a fixed path
        path = 'C:\SynBuckConverter-PSpiceFiles'
    if file_name is None:
        # use this for setting or change a fixed file name
        file_name = 'test.csv'
        
    if file_name[-4:] != '.csv':
        raise NameError('File_name needs to end with .csv')
           
    if type(path) is not str or type(file_name) is not str:
        raise TypeError("Path and name needs to be strings!")
        
    # Prints values in columns instead of row
    data_to_save = zip_longest(*data_to_save, fillvalue = '')
    
     #Notes:
    # newline = '' --> no free column duriing    
    # delimter = ',' --> Gibt Treenung der Variablen in der Liste an        
    with open((path + '\\' + file_name), 'w', encoding = 'utf8', newline ='') as f:
        writer = csv.writer(f, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        
        for data in data_to_save:
            writer.writerow(data)
    
    # Zeigt File an indem Datei Gespeichert wurde
    print(path + file_name)