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

ref = 8

# --- Creates buck object ---
buck = cpe.hw.buck.Buck(COM, baud, to)

# --- Sets up data for experiments ---
# SFB data
sfb_params = {'k_il':0.09112457, 'k_vc':0.20602978, 'k_z':935.0232993623816, 'dt':1/200e3}

# Luenberger params
luenberger_params = {'a11':0.99219015, 'a12':-1.11785885, 'a21':0.0076528, 'a22':0.85029795, 'b11':1.64135135, 'b12':1.0195765, 'b21':0.09610925, 'b22':0.13657431}

