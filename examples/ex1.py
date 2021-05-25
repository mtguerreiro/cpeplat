import cpeplat as cpe
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
COM = 'COM12'
baud = 115200
to = 1

# --- Connection ---
plat = cpe.interface.Interface(COM, baud, to)

# --- Commands ---

d = plat.adcConv()
#d = plat.adcRead()
 
plt.plot(d)
