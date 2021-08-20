import cpeplat as cpe
import matplotlib.pyplot as plt
import numpy as np

import time

plt.ion()

# --- Input ---
COM = 'COM12'
baud = 115200
to = 3

ref = 8

# --- Creates buck object ---
buck = cpe.hw.buck.Buck(COM, baud, to)


# --- Controller data ---
# SFB data
# Ts = 1.5e-3, p_os = 2
sfb_params_1 = {'k_il':0.07755398, 'k_vc':0.1141189, 'k_z':541.1014463940613, 'dt':1/50e3}
# Ts = 1.25e-3, p_os = 2
sfb_params_2 = {'k_il':0.09112457, 'k_vc':0.20602978, 'k_z':935.0232993623816, 'dt':1/50e3}
# Ts = 1.25e-3, p_os = 0.1
sfb_params_3 = {'k_il':0.09108946, 'k_vc':0.20664688, 'k_z':686.0082703034772, 'dt':1/50e3}

# Ts = 1.5e-3, p_os = 5
sfb_params_4 = {'k_il':0.07751897, 'k_vc':0.11473426, 'k_z':690.7245259710837, 'dt':1/50e3}
# Ts = 1.25e-3, p_os = 5
sfb_params_5 = {'k_il':0.09116102, 'k_vc':0.20538905, 'k_z':1193.5719808745957, 'dt':1/50e3}

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
cimini_params = cimini(1.1, 47e-6, 470e-6, 25e-3, 25e-3, 1/50e3, 1, 0.5, 10)

# Luenberger params
#luenberger_params = {'a11':0.92879781, 'a12':-3.55398592, 'a21':0.02433039, 'a22':0.47768281, 'b11':5.8777473, 'b12':3.24133709, 'b21':0.36903981, 'b22':0.47650462}
luenberger_params = {'a11':0.96246115, 'a12':-1.50947508, 'a21':0.02724377, 'a22':0.65462411, 'b11':6.38834801, 'b12':1.13344032, 'b21':0.41322953, 'b22':0.29407761}

# --- Useful commands ---

#u = np.array(buck.plat.cpu2_buffer_read(0))
#niters = np.array(buck.plat.cpu2_buffer_read(1))
