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
# Open-loop data
ol_params = {'u': 125/499}

# PID data
pid_params_1 = {'a1':-1.6327, 'a2':0.6327, 'b0':1.5703, 'b1':-3.1017, 'b2':1.5321}
pid_params_2 = {'a1':-1.77777, 'a2':0.7777, 'b0':5.756944444444445, 'b1':-11.19999938888889, 'b2':5.445833333333333}

# SFB data
sfb_params = {'k_il':0.09437662/2, 'k_vc':0.60625809/2, 'k_z':6020.401591307597/2, 'dt':1/200e3}

controllers = [['ol', ol_params], ['pid', pid_params_1], ['pid', pid_params_2], ['sfb', sfb_params]]

# --- Runs experiments with all parameters ---

data = []
for ctl in controllers:
    data.append(buck.experiment(ref, ctl[0], ctl[1]))
