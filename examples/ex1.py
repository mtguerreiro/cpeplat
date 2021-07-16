import cpeplat as cpe
import matplotlib.pyplot as plt
import numpy as np

import time

plt.ion()

# --- Input ---
COM = 'COM12'
baud = 115200
to = 1

# --- Connection and settings ---
plat = cpe.interface.Interface(COM, baud, to)

# Changes blinking rate for fun
plat.cpu1_blink(250)
plat.cpu2_blink(250)

# Sets ADC buffers
plat.cpu1_adc_set_buffer(0, 3000)
plat.cpu1_adc_set_buffer(1, 3000)
plat.cpu1_adc_set_buffer(2, 3000)

# --- Experiment ---
def experiment():
    # Turns on the input relay, waits and then turns on the output
##    plat.cpu2_gpio(8, 1)
##    time.sleep(1)
##    plat.cpu2_gpio(9, 1)
##    time.sleep(1)

    # Enables the PWM signal, waits a little bit, then disables it
    plat.cpu2_pwm_enable()
    time.sleep(1)
    plat.cpu2_pwm_disable()
    time.sleep(1)

    # Turns off the input and output relays
##    plat.cpu2_gpio(8, 0)
##    time.sleep(1)
##    plat.cpu2_gpio(9, 0)
##    time.sleep(1)

    # Reads ADC measurements
    v_in = plat.cpu1_adc_read_buffer(0)
    v_in = np.array(v_in)

    il = plat.cpu1_adc_read_buffer(1)
    il = np.array(il)

    v_out = plat.cpu1_adc_read_buffer(2)
    v_out = np.array(v_out)

    return v_in, v_out, il
