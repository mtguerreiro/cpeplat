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
#ol_params = {'u': 125/499}
ol_params = {'u': 600/2000}

# PID data
pid_params_1 = {'a1':-1.6327, 'a2':0.6327, 'b0':1.5703, 'b1':-3.1017, 'b2':1.5321}
pid_params_2 = {'a1':-1.77777, 'a2':0.7777, 'b0':5.756944444444445, 'b1':-11.19999938888889, 'b2':5.445833333333333}

# SFB data
sfb_params = {'k_il':0.09437662/2, 'k_vc':0.60625809/2, 'k_z':6020.401591307597/2, 'dt':1/50e3}

controllers = [['ol', ol_params], ['pid', pid_params_1], ['pid', pid_params_2], ['sfb', sfb_params]]


#u = np.array(buck.plat.cpu2_buffer_read(0))
#niters = np.array(buck.plat.cpu2_buffer_read(1))

# --- Runs experiments with all parameters ---

##data = []
##for ctl in controllers:
##    data.append(buck.experiment(ref, ctl[0], ctl[1]))


# --- Observer ---
def cimini(R, L, C, RL, Rds, ts, rho, alpha, K):
    a11 = 1 - ts * (RL + Rds) / L
    a12 = -ts / L
    b11 = ts / L

    a21 = ts / C
    a22 = 1 - ts / (R * C) - ts * K / C
    a23 = ts * K / C
    a24 = -ts / C
    a25 = rho
    a26 = alpha

    return {'a11':a11, 'a12':a12, 'b11':b11, 'a21':a21, 'a22':a22, 'a23':a23, 'a24':a24, 'a25':a25, 'a26':a26}

# Observer parameters
observer = 'cimini'
#obs_params = cimini(1.1, 47e-6, 470e-6, 25e-3, 25e-3, 1/50e3, 0.78, 0.5, 10)
obs_params = cimini(1.1, 60e-6, 470e-6, 25e-3, 25e-3, 1/50e3, 0.78, 0.5, 10)
