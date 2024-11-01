# -*- coding: utf-8 -*-
"""
Module ``result``
===========================

This module contains the functions for plotting and saving the data from an experiment.

"""




import cpeplat as cpe
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time
import cpeplat.result as res
from itertools import zip_longest
import csv

plt.style.use('seaborn-colorblind')
matplotlib.rcParams['mathtext.fontset'] = 'stixsans'
matplotlib.rcParams['font.family'] = 'Latin Modern Sans'
matplotlib.rcParams.update({'font.size': 12})
plt.rc('axes', unicode_minus=False)
plt.ion()
l_fs = 12
title_fs = 12.5
lw = 1.5

##########-------------- Basic Setting Functions  ----------------############

def _setting_labels(type_of):
    if type_of == 'Voltage':
        plt.xlabel("Time (ms)", fontsize=l_fs)
        plt.ylabel("Voltage (V)", fontsize=l_fs)  
    
    elif type_of == 'Current':
        plt.xlabel("Time (ms)", fontsize=l_fs)
        plt.ylabel("Current (A)", fontsize=l_fs) 
        
    elif type_of == 'ControlSignal':
        plt.xlabel("Time (ms)", fontsize=l_fs)
        plt.ylabel("Duty cycle", fontsize=l_fs)
##    if type_of == 'Voltage':
##        plt.xlabel("Time / ms", fontsize=12)
##        plt.ylabel("Voltage / V", fontsize=12)  
##    
##    elif type_of == 'Current':
##        plt.xlabel("Time / ms", fontsize=12)
##        plt.ylabel("Current / A", fontsize=12) 
##        
##    elif type_of == 'ControlSignal':
##        plt.xlabel("Time / ms", fontsize=12)
##        plt.ylabel("u (DutyCycle)", fontsize=12)        
        
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
    plt.title(title, fontsize=title_fs)


##########-------------- Plot Functions  -------------############

def plot_vin(data, leg = None):
    """Plots input voltage from saved experiment data.

    Example
    --------------
    Code Example for 1 and more data sets.
          
    .. code-block:: python
    
       res.plot_vin([data1],['pid_1']) 
       res.plot_vin([data1, data2],['pid_1', 'pid_2'])
       
    Parameters
    --------------
    
    data : list of lists
        Saved data from experiments.

    leg : list of strings
        Generating the naming for the legend.
        
       
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

def plot_vin_buck(data, leg = None):
    """Plots input voltage at buck converter side from saved experiment data.

    Example
    --------------
    Code Example for 1 and more data sets.
          
    .. code-block:: python
    
        res.plot_vin_buck([data1],['pid_1']) 
        res.plot_vin_buck([data1, data2],['pid_1', 'pid_2'])
       
    Parameters
    --------------
    
    data : list of lists
        Saved data from experiments.

    leg : list of strings
        Generating the naming for the legend.
        
       
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
    
def plot_vout(data, leg = None):
    """Plots output voltage from saved experiment data.

    Example
    --------------
    Code Example for 1 and more data sets.
          
    .. code-block:: python
    
        res.plot_vout([data1],['pid_1']) 
        res.plot_vout([data1, data2],['pid_1', 'pid_2'])
       
    Parameters
    --------------
    
    data : list of lists
        Saved data from experiments.

    leg : list of strings
        Generating the naming for the legend.
        
       
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

    
def plot_vout_buck(data, leg = None):
    """Plots outout voltage at buck converter side (before Load) from saved experiment data.

    Example
    --------------
    Code Example for 1 and more data sets.
          
    .. code-block:: python
    
        res.plot_vout_buck([data1],['pid_1']) 
        res.plot_vout_buck([data1, data2],['pid_1', 'pid_2'])
       
    Parameters
    --------------
    
    data : list of lists
        Saved data from experiments.

    leg : list of strings
        Generating the naming for the legend.
        
       
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
    
def plot_il(data, leg = None):
    """Plots inductance current from saved experiment data.

    Example
    --------------
    Code Example for 1 and more data sets.
          
    .. code-block:: python
    
        res.plot_il([data1],['pid_1']) 
        res.plot_il([data1, data2],['pid_1', 'pid_2'])
       
    Parameters
    --------------
    
    data : list of lists
        Saved data from experiments.

    leg : list of strings
        Generating the naming for the legend.
        
       
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
    
def plot_il_avg(data, leg = None):
    """Plots average inductance current from saved experiment data.

    Example
    --------------
    Code Example for 1 and more data sets.
          
    .. code-block:: python
    
        res.plot_il_avg([data1],['pid_1']) 
        res.plot_il_avg([data1, data2],['pid_1', 'pid_2'])
       
    Parameters
    --------------
    
    data : list of lists
        Saved data from experiments.

    leg : list of strings
        Generating the naming for the legend.
        
       
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
    
def plot_u(data, leg = None):
    """Plots control signal (Duty Cycle) from saved experiment data.

    Example
    --------------
    Code Example for 1 and more data sets.
          
    .. code-block:: python
    
        res.plot_u([data1],['pid_1']) 
        res.plot_u([data1, data2],['pid_1', 'pid_2'])
       
    Parameters
    --------------
    
    data : list of lists
        Saved data from experiments.

    leg : list of strings
        Generating the naming for the legend.
        
       
    """
    if len(data) == 1:
        print('yes')
        
    plt.figure(figsize=(9, 6))
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[7])
        counter_data = counter_data + 1
        
    
    res._setting_labels('ControlSignal')
    res._setting_title('Control Signal (DutyCycle)')
    res._setting_legend(leg, counter_data)
    plt.grid()   
    
def plot_all(data, leg = None, title = None):
    """Plots output voltage, inductance current and control signal from saved experiment data.

    Example
    --------------
    Code Example for 1 and more data sets.
          
    .. code-block:: python
    
       res.plot_all([data1],['pid_1']) 
       res.plot_all([data1, data2],['pid_1', 'pid_2'], 'Comparision Cascaded and PID Control')
       
    Parameters
    --------------
    
    data : list of lists
        Saved data from experiments.

    leg : list of strings
        Generating the naming for the legend.
        
    title : str
        Title for the plot.
        
       
    """


    plt.figure(figsize=(8, 7))
    plt.suptitle(title)
    ax = plt.subplot(3, 1, 1)
    
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[3], lw=lw)
        counter_data = counter_data + 1
        
    res._setting_labels('Voltage')
    res._setting_title('Output voltage')
    res._setting_legend(leg, counter_data)
    plt.grid()   
    
    plt.subplot(3, 1, 2, sharex=ax)
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[6], lw=lw)
        counter_data = counter_data + 1
        
    res._setting_labels('Current')
    res._setting_title('Average inductor current')
    res._setting_legend(leg, counter_data)
    plt.grid() 
    
    plt.subplot(3, 1, 3, sharex=ax)
    counter_data = 0
    for i in data:
        plt.plot(i[0],i[7], lw=lw)
        counter_data = counter_data + 1
        
    res._setting_labels('ControlSignal')
    res._setting_title('Control signal')
    res._setting_legend(leg, counter_data)
    plt.grid()

    plt.tight_layout()
    
    
def save_variable_csv(data_to_save, path = None, file_name = None):
    """Saves data from an experiment in a csv-file.

    Example
    --------------
    Important: Place a 'r' before the 'path' string and file name needs to end with .csv
          
    .. code-block:: python
    
        res.save_variable_csv(data,r'path','file_name.csv') 
       
    Parameters
    --------------
    
    data : list of lists
        Saved data from experiments.

    path : raw string 
        Path to the saving location.
    
    file_name : str
        file name with .csv at the end.
        
       
    """    
    
    if path is None:
        # use this for setting a fixed path
        path = r'C:\Users'
    if file_name is None:
        # use this for setting or change a fixed file name
        file_name = 'testq.csv'
        
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
    print(path + '\\' + file_name)
