import cpeplat as cpe
import matplotlib.pyplot as plt
import numpy as np

plt.ion()

# --- Input ---
path = 'C:/Users/mguerreiro/Google Drive/Doutorado/Estudos iniciais/Buck converter/PLECS/v3/open-loop'
file = 'buck-open-loop'

# --- Simulation ---
buck = cpe.plecs.buck.Buck()

model_params = buck.model.params
#model_params['V_ref'] = 12
