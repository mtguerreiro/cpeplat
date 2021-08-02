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
# PID data
control_pid = 'pid'
params_pid = {'a1':-1.6327, 'a2':0.6327, 'b0':1.5703, 'b1':-3.1017, 'b2':1.5321}

control_pid2 = 'pid'
params_pid2 = {'a1':-1.77777, 'a2':0.7777, 'b0':5.756944444444445, 'b1':-11.19999938888889, 'b2':5.445833333333333}

# OL data
dc = 125/499
control_ol = 'ol'
params_ol = {'u':dc}

# SFB data
control_sfb = 'sfb'
params_sfb = {'k_il':0.09437662/2, 'k_vc':0.60625809/2, 'k_z':6020.401591307597/2, 'dt':1/100e3}

# SFB data
control_sfb = 'sfb'
params_sfb = {'k_il':0.21797013/2, 'k_vc':0.63250998/2, 'k_z':3536.9859347697425/2, 'dt':1/100e3}


##data = []
##for ctl, params in zip([control_pid1, control_ol, control_sfb], [params_pid1, params_ol, params_sfb]):
##    data.append(buck.experiment(ref, ctl, params))


### Observer parameters
##def cimini(R, L, C, RL, Rds, ts, rho, alpha, K):
##    a11 = 1 - ts * (RL + Rds) / L
##    a12 = -ts / L
##    b11 = ts / L
##
##    a21 = ts / C
##    a22 = 1 - ts / (R * C) - ts * K / C
##    a23 = ts * K / C
##    a24 = -ts / C
##    a25 = rho
##    a26 = alpha
##
##    return {'a11':a11, 'a12':a12, 'b11':b11, 'a21':a21, 'a22':a22, 'a23':a23, 'a24':a24, 'a25':a25, 'a26':a26}
##
##observer = 'cimini'
##params_obs = cimini(1.1, 47e-6, 470e-6, 15e-3, 15e-3, 1/200e3, 0.78, 100, 30)
##
##for K in [100, 50, 30]:
##    params_obs = cimini(1.1, 47e-6, 470e-6, 15e-3, 15e-3, 1/200e3, 0.78, 100, K)
##    data.append(buck.experiment(ref, 'pid', params_pid1, obs='cimini', obs_params=params_obs))
##    il.append(np.array(buck.plat.cpu2_buffer_read_float(1)))
##    vc.append(np.array(buck.plat.cpu2_buffer_read_float(2)))
