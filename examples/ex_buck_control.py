"""This is an example on how to run an experiment with the buck converter.

To run an experiment, run the command:

>>> data_pid_fast = buck.experiment(8, 'pid', pid_params_fast)

Here, 8 is the reference, 'pid' is the controller and pid_params_fast is the
controller type. The experimental data is saved in the data_pid_fast variable.

To plot the experimental data, run the commnad:

>>> res.plot_compare_all([data_pid_fast])

You can change the controller and the parameters. The basic structure to set
the controller parameters is given in the examples below.

"""
import cpeplat as cpe
import matplotlib.pyplot as plt
import numpy as np
import cpeplat.result as res

import time

plt.ion()

# --- Input ---
COM = 'COM12'
baud = 115200
to = 3

# --- Creates buck object ---
buck = cpe.hw.buck.Buck(COM, baud, to)

# --- Sets up data for experiments ---
# Open-loop data
ol_params = {'u': 125/499}

# PID data
pid_params_slow = {'a1':-1.77777, 'a2':0.7777, 'b0':5.756944444444445, 'b1':-11.19999938888889, 'b2':5.445833333333333}
pid_params_fast = {'a1':-1.63265312, 'a2':0.632653058, 'b0':1.57026279, 'b1':-3.10171938, 'b2':1.53209949}

# SFB data
sfb_params = {'k_il':0.09437662/2, 'k_vc':0.60625809/2, 'k_z':6020.401591307597/2, 'dt':1/200e3}
