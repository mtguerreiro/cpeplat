import cpeplat as cpe
import matplotlib.pyplot as plt
import numpy as np

import time

plt.ion()

# --- Input ---
COM = 'COM12'
baud = 115200
to = 3

ref = 650

# --- Creates buck object ---
buck = cpe.hw.buck.Buck(COM, baud, to)


# --- Sets up data for experiments ---
# PID experiment
control_pid1 = 'pid'
params_pid1 = {'a1':-1.6327, 'a2':0.6327, 'b0':1.5703, 'b1':-3.1017, 'b2':1.5321}
#data_pid = buck.experiment(ref, control_pid, params_pid)

control_pid2 = 'pid'
params_pid2 = {'a1':-1.77777, 'a2':0.7777, 'b0':5.756944444444445, 'b1':-11.19999938888889, 'b2':5.445833333333333}
#data_pid = buck.experiment(ref, control_pid, params_pid)

# OL experiment
dc = 0.25
control_ol = 'ol'
params_ol = {'u':dc}
#data_ol = buck.experiment(ref, control_ol, params_ol)

