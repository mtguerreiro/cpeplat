import cpeplat as cpe
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
COM = 'COM12'
baud = 115200
to = 1

# --- Connection ---
plat = cpe.interface.Interface(COM, baud, to)

plat.cpu1_blink(250)
plat.cpu2_blink(250)

