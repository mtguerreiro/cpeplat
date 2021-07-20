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
plat.cpu1_adc_set_buffer(0, 1000)
plat.cpu1_adc_set_buffer(1, 1000)
plat.cpu1_adc_set_buffer(2, 1000)
plat.cpu1_adc_set_buffer(3, 1000)
plat.cpu1_adc_set_buffer(4, 1000)
plat.cpu1_adc_set_buffer(5, 1000)

# --- Experiment ---
def experiment(dc):
    # Turns on the input relay, waits and then turns on the output
    plat.cpu2_gpio(8, 1)
    time.sleep(1)
    plat.cpu2_gpio(9, 1)
    time.sleep(1)

    # Enables the PWM signal, waits a little bit, then disables it
    plat.cpu2_pwm_enable(dc)
    time.sleep(5)
    plat.cpu2_pwm_disable()
    time.sleep(1)

    # Turns off the input and output relays
    plat.cpu2_gpio(8, 0)
    time.sleep(1)
    plat.cpu2_gpio(9, 0)
    time.sleep(1)

    v_in = plat.cpu1_adc_read_buffer(0)
    v_in = np.array(v_in)

    v_in_buck = plat.cpu1_adc_read_buffer(1)
    v_in_buck = np.array(v_in_buck)

    v_out = plat.cpu1_adc_read_buffer(2)
    v_out = np.array(v_out)

    v_out_buck = plat.cpu1_adc_read_buffer(3)
    v_out_buck = np.array(v_out_buck)
    
    il = plat.cpu1_adc_read_buffer(4)
    il = np.array(il)

    il_avg = plat.cpu1_adc_read_buffer(5)
    il_avg = np.array(il_avg)

    return v_in, v_in_buck, v_out, v_out_buck, il, il_avg


def relays_on():
    plat.cpu2_gpio(8, 1)
    time.sleep(0.25)
    plat.cpu2_gpio(9, 1)
    time.sleep(0.25)


def relays_off():
    plat.cpu2_gpio(8, 0)
    time.sleep(0.25)
    plat.cpu2_gpio(9, 0)
    time.sleep(0.25)


def read_adcs():
    # Reads ADC measurements
    v_in = plat.cpu1_adc_read_buffer(0)
    v_in = np.array(v_in)

    v_in_buck = plat.cpu1_adc_read_buffer(1)
    v_in_buck = np.array(v_in_buck)

    v_out = plat.cpu1_adc_read_buffer(2)
    v_out = np.array(v_out)

    v_out_buck = plat.cpu1_adc_read_buffer(3)
    v_out_buck = np.array(v_out_buck)
    
    il = plat.cpu1_adc_read_buffer(4)
    il = np.array(il)

    il_avg = plat.cpu1_adc_read_buffer(5)
    il_avg = np.array(il_avg)

    return v_in, v_in_buck, v_out, v_out_buck, il, il_avg

#plt.plot(vi / 4095 * 3 * 10)
#plt.plot(((il / 4095 * 3) * (3.9+2)/3.9 - 2.5) / 50e-3)
