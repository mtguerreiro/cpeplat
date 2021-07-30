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

#plat.ser.serial.set_buffer_size(65535)

# Changes blinking rate for fun
plat.cpu1_blink(2000)
plat.cpu2_blink(2000)

# Sets ADC buffers
plat.cpu1_adc_buffer_set(0, 2000)
plat.cpu1_adc_buffer_set(1, 2000)
plat.cpu1_adc_buffer_set(2, 2000)
plat.cpu1_adc_buffer_set(3, 2000)
plat.cpu1_adc_buffer_set(4, 2000)
plat.cpu1_adc_buffer_set(5, 2000)

plat.cpu2_buffer_set(0, 2000)
plat.cpu2_buffer_set(1, 2000)
plat.cpu2_buffer_set(2, 2000)

# Sets tripping values for all ADCs
plat.cpu2_trip_set(0, 3000)
plat.cpu2_trip_set(1, 3000)
plat.cpu2_trip_set(2, 3000)
plat.cpu2_trip_set(3, 3000)
plat.cpu2_trip_set(4, 3000)
plat.cpu2_trip_set(5, 3000)

plat.cpu2_trip_enable(0)
plat.cpu2_trip_enable(1)
plat.cpu2_trip_enable(2)
plat.cpu2_trip_enable(3)
plat.cpu2_trip_enable(4)
plat.cpu2_trip_enable(5)

# --- Experiment ---
def experiment_ol(dc):
    # Sets control mode to open loop
    plat.cpu2_control_mode_set('ol', {'u':dc})
    
    # Turns on the input relay, waits and then turns on the output
    plat.cpu2_gpio(8, 1)
    time.sleep(0.5)
    plat.cpu2_gpio(9, 1)
    time.sleep(0.5)

    # Enables the PWM signal, waits a little bit, then disables it
    plat.cpu2_pwm_enable()
    time.sleep(2)
    plat.cpu2_pwm_disable()
    time.sleep(0.5)

    # Turns off the input and output relays
    plat.cpu2_gpio(8, 0)
    time.sleep(0.5)
    plat.cpu2_gpio(9, 0)
    time.sleep(0.5)

def experiment_pid(ref):
    # Sets ref
    plat.cpu2_ref_set(ref)

    # Sets control mode to pid
    #plat.cpu2_control_mode_set('pid',{'a1':-1.77777, 'a2':0.7777, 'b0':5.756944444444445, 'b1':-11.19999938888889, 'b2':5.445833333333333})
    plat.cpu2_control_mode_set('pid',{'a1':-1.6327, 'a2':0.6327, 'b0':1.5703, 'b1':-3.1017, 'b2':1.5321})
    
    # Turns on the input relay, waits and then turns on the output
    plat.cpu2_gpio(8, 1)
    time.sleep(0.5)
    plat.cpu2_gpio(9, 1)
    time.sleep(0.5)

    # Enables the PWM signal, waits a little bit, then disables it
    plat.cpu2_pwm_enable()
    time.sleep(2)
    plat.cpu2_pwm_disable()
    time.sleep(0.5)

    # Turns off the input and output relays
    plat.cpu2_gpio(8, 0)
    time.sleep(0.5)
    plat.cpu2_gpio(9, 0)
    time.sleep(0.5)

def read_data():
    
    # Reads ADC buffers
    v_in = plat.cpu1_adc_buffer_read(0)
    v_in = np.array(v_in)

    v_in_buck = plat.cpu1_adc_buffer_read(1)
    v_in_buck = np.array(v_in_buck)

    v_out = plat.cpu1_adc_buffer_read(2)
    v_out = np.array(v_out)

    v_out_buck = plat.cpu1_adc_buffer_read(3)
    v_out_buck = np.array(v_out_buck)
    
    il = plat.cpu1_adc_buffer_read(4)
    il = np.array(il)

    il_avg = plat.cpu1_adc_buffer_read(5)
    il_avg = np.array(il_avg)

    
    # Reads CPU2 buffer
    u = plat.cpu2_buffer_read(0)

    data = [v_in, v_in_buck, v_out, v_out_buck, il, il_avg, u]

    return data

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
    v_in = plat.cpu1_adc_buffer_read(0)
    v_in = np.array(v_in)
    time.sleep(0.1)

    v_in_buck = plat.cpu1_adc_buffer_read(1)
    v_in_buck = np.array(v_in_buck)
    time.sleep(0.1)

    v_out = plat.cpu1_adc_buffer_read(2)
    v_out = np.array(v_out)
    time.sleep(0.1)

    v_out_buck = plat.cpu1_adc_buffer_read(3)
    v_out_buck = np.array(v_out_buck)
    time.sleep(0.1)
    
    il = plat.cpu1_adc_buffer_read(4)
    il = np.array(il)
    time.sleep(0.1)

    il_avg = plat.cpu1_adc_buffer_read(5)
    il_avg = np.array(il_avg)
    time.sleep(0.1)

    return v_in, v_in_buck, v_out, v_out_buck, il, il_avg

#plt.plot(vi / 4095 * 3 * 10)
#plt.plot(((il / 4095 * 3) * (3.9+2)/3.9 - 2.5) / 50e-3)


#plat.cpu2_control_mode_set('pid',{'a1':0.35, 'a2':0.257, 'b0':-0.2, 'b1':2.2, 'b2':-0.9})
#plat.cpu2_control_mode_set('pid',{'a1':-1.77777, 'a2':0.7777, 'b0':5.756944444444445, 'b1':-11.19999938888889, 'b2':5.445833333333333})
#plat.cpu2_control_mode_set('ol',{'u':0.1})

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
