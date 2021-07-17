import cpeplat as cpe
import matplotlib.pyplot as plt
import numpy as np

import time

plt.ion()

# --- Input ---
COM = 'COM12'
baud = 115200
to = 3

# --- Connection and settings ---
plat = cpe.interface.Interface(COM, baud, to)

plat.ser.serial.set_buffer_size(65535)

# Changes blinking rate for fun
plat.cpu1_blink(250)
plat.cpu2_blink(250)

# Sets ADC buffers
plat.cpu1_adc_set_buffer(0, 3000)
plat.cpu1_adc_set_buffer(1, 3000)
plat.cpu1_adc_set_buffer(2, 3000)

# --- Experiment ---
def experiment(dc):
    # Turns on the input relay, waits and then turns on the output
    plat.cpu2_gpio(8, 1)
    time.sleep(1)
    plat.cpu2_gpio(9, 1)
    time.sleep(1)

    # Enables the PWM signal, waits a little bit, then disables it
    plat.cpu2_pwm_enable(dc)
    time.sleep(2)
    plat.cpu2_pwm_disable()
    time.sleep(1)

    # Turns off the input and output relays
    plat.cpu2_gpio(8, 0)
    time.sleep(1)
    plat.cpu2_gpio(9, 0)
    time.sleep(1)

    # Reads ADC measurements
    v_in = plat.cpu1_adc_read_buffer(0)
    v_in = np.array(v_in)

    #time.sleep(0.1)

    il = plat.cpu1_adc_read_buffer(1)
    il = np.array(il)

    #time.sleep(0.1)

    v_out = plat.cpu1_adc_read_buffer(2)
    v_out = np.array(v_out)

    #time.sleep(0.1)

    return v_in, v_out, il


##vin = []; vout = []; il = [];
##for k in range(5):
##	print(k)
##	vi, vo, i = experiment(0.)
##	vin.append(vi)
##	vout.append(vo)
##	il.append(i)
##	time.sleep(0.5)

##for a in vout:
##    plt.plot(a)
##
##for a in vin:
##    plt.plot(a)
##
##for a in il:
##    plt.plot(a)
