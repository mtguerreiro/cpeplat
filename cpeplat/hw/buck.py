import cpeplat as cpe
import numpy as np
import time

class BuckHWM:
    """A class to hold hardware mapping data for the buck platform.
    """
    def __init__(self):
        self.adc_vin = 0
        self.adc_vin_buck = 1
        self.adc_il = 2
        self.adc_vout = 3
        self.adc_il_avg = 4
        self.adc_vout_buck = 5
        
        self.cpu2_buffer_u = 0

        self.gpio_input_relay = 8
        self.gpio_output_relay = 9
        self.gpio_fault_enable = 10
        
        self.vref_limit = 30
        self.adc_Resolution = 4095
        self.sample_time = 5e-06
        
        self.il_sensor_sensitiviy = 50e-03
        self.il_sensor_offset = -2.5
        self.il_resistor_gain = 3.9/5.9
        self.il_adc_voltage_gain = 3/4095
        self.u_pwm_gain = 0x03E7>>1
        
        
        self.vin_buck_offset = 0
        self.vin_offset = 0
        self.vout_offset = 0
        self.vout_buck_offset = 0 
        self.il_offset = self.il_sensor_offset/self.il_sensor_sensitiviy
        self.il_avg_offset = self.il_sensor_offset/self.il_sensor_sensitiviy
        self.u_offset = 0
        
        self.vin_buck_gain = self.vref_limit/4095
        self.vin_gain = self.vref_limit/4095
        self.vout_gain = self.vref_limit/4095
        self.vout_buck_gain = self.vref_limit/4095 
        self.il_gain = self.il_adc_voltage_gain/(self.il_resistor_gain*self.il_sensor_sensitiviy)
        self.il_avg_gain = self.il_adc_voltage_gain/(self.il_resistor_gain*self.il_sensor_sensitiviy)
        self.u_gain = 1/self.u_pwm_gain


class BuckHWDefaultSettings:
    """A class to hold default hardware settings for the buck platform.
    """
    def __init__(self):

        # Buffers
        self.vin_buffer_size = 2000
        self.vin_buck_buffer_size = 2000

        self.il_buffer_size = 2000
        self.vout_buffer_size = 2000

        self.il_avg_buffer_size = 2000
        self.vout_buck_buffer_size = 2000

        self.u_buffer_size = 2000

        # Tripping
        self.vin_trip = 22
        self.vin_buck_trip = 22

        self.vout_trip = 22
        self.vout_buck_trip = 22

        self.il_trip = 20
        self.il_avg_trip = 10

        # Blinking rate
        self.cpu1_blink = 2000
        self.cpu2_blink = 2000

        
class Buck:
    """A class to provide an interface to the buck hardware, via the C2000
    platform.

    Hardware mapping
    ================

    The buck hardware contains 6 analog measurements, two PWM signals and
    three GPIOs. Here, we document how these signals are mapped to the C2000
    board.

    Analog measurements
    -------------------

    The analog measurements available are:

    - V_in: Input voltage, before input relay.
    - V_in_buck: Input voltage, after input relay.
    - V_out: Output voltage, after output relay.
    - V_out_buck: Output voltage, before output relay.
    - IL: Inductor current.
    - IL_avg: Filtered inductor current.

    The ADCs measuring these signals are:
    
    - ADC_A1: Vin
    - ADC_A4: Vin_buck
    - ADC_A5: IL
    - ADC_B4: Vout
    - ADC_B5: IL_avg
    - ADC_C4: Vout_buck

    From the :class:`cpe.interface.Interface` class, we can check which ADC
    index corresponds to each ADC. Currently, they are:
    
    - ADC 0: Vin
    - ADC 1: Vin_buck
    - ADC 2: IL
    - ADC 3: Vout
    - ADC 4: IL_avg
    - ADC 5: Vout_buck
    
    PWM signals
    -----------

    The buck hardware requires two PWM signals, complementary to each other.
    Currently, the PWM signals from the C2000 board are EPWM4A and EPWM4B.

    GPIOs
    -----

    The buck hardware has two relays (input and output), and one fault signal.
    In this version, the GPIOs corresponding to each function are initialized
    in the firmware, and the interface only allows setting/resetting these
    pins.

    The GPIOs from the C2000 board connected to these signals are:

    - GPIO 8: Input relay
    - GPIO 9: Output relay
    - GPIO 10: Fault enable.

    Class documentation
    ===================

    Parameters
    ----------
    com : str
        COM port.

    baud : int
        Baudrate used for communication. By default, it is 115200 bps.

    timeout : int, float
        Communication time-out, in seconds. By default, it is 0.2 s.

    Raises
    ------
    ValueError
        If setting any of the tripping values fail.
        
    """
    def __init__(self, com, baud=115200, to=0.2):

        plat = cpe.interface.Interface(com, baud, to)
        self.plat = plat
        
        hwm = BuckHWM()
        self.hwm = hwm

        hw_default = BuckHWDefaultSettings()
        self.hw_default = hw_default

        # Initializes ADC/CPU2 buffers
        plat.cpu1_adc_buffer_set(hwm.adc_vin, hw_default.vin_buffer_size)
        plat.cpu1_adc_buffer_set(hwm.adc_vin_buck, hw_default.vin_buck_buffer_size)
        
        plat.cpu1_adc_buffer_set(hwm.adc_vout, hw_default.vout_buffer_size)
        plat.cpu1_adc_buffer_set(hwm.adc_vout_buck, hw_default.vout_buck_buffer_size)
        
        plat.cpu1_adc_buffer_set(hwm.adc_il, hw_default.il_buffer_size)
        plat.cpu1_adc_buffer_set(hwm.adc_il_avg, hw_default.il_avg_buffer_size)

        plat.cpu2_buffer_set(hwm.cpu2_buffer_u, hw_default.u_buffer_size)

        # Sets and enable tripping for all ADCs
        status = 0
        status = self._set_trip_vin(hw_default.vin_trip)

        status |= self._set_trip_vin_buck(hw_default.vin_buck_trip)
        
        status |=  self._set_trip_vout(hw_default.vout_trip)
        status |=  self._set_trip_vout_buck(hw_default.vout_buck_trip)
        
        status |= self._set_trip_il(hw_default.il_trip)
        status |= self._set_trip_il_avg(hw_default.il_avg_trip)

        status |= plat.cpu2_trip_enable(hwm.adc_vin)
        status |= plat.cpu2_trip_enable(hwm.adc_vin_buck)
        status |= plat.cpu2_trip_enable(hwm.adc_vout)
        status |= plat.cpu2_trip_enable(hwm.adc_vout_buck)
        status |= plat.cpu2_trip_enable(hwm.adc_il)
        status |= plat.cpu2_trip_enable(hwm.adc_il_avg)

        if status != 0:
            raise ValueError('Error setting the tripping limits for the ADCs. Cannot continue.')

        # Changes blinking rate for fun
        plat.cpu1_blink(hw_default.cpu1_blink)
        plat.cpu2_blink(hw_default.cpu2_blink)


    def read_status(self):
        """Gets the status of the controller.

        In this case, the status of the controller is the status of CPU2.

        The status is a positive integer (0 included) if the status was read
        correctly. The `plat_defs.h` file of the firmware code contains the
        description of each bit of the status flag. If there was any error
        reading the status, -1 is returned.

        Returns
        -------
        status : int
            Status of controller.
            
        """
        status = self.plat.cpu2_status()

        if status < 0:
            print('\nError reading status. Error code: {:}'.format(status))
            status = -1
            
        return status

        
    def clear_status(self):
        """Clears the status of the controller.

        In this case, the status of the controller is the status of CPU2.

        This function returns 0 if the status was cleared and -1 otherwise.

        Returns
        -------
        status : int
            Returns 0 if status was cleared, and -1 if an error occurred.
            
        """
        status = self.plat.cpu2_status_clear()

        if status != 0:
            print('\nError clearing status. Error code: {:}'.format(status))
            status = -1
            
        return status
    

    def disable_input_relay(self):
        """Opens the input relay, disconnecting the input voltage from the
        power circuit.

        On the hardware, the input voltage is still connected to the power
        circuit through the pre-charge resistors.

        Returns
        -------
        status : int
            Returns 0 if command was executed properly, -1 otherwise.
        
        """
        gpio = self.hwm.gpio_input_relay
        
        status = self.plat.cpu2_gpio(gpio, 0)

        if status != 0:
            print('\nError opening the input relay. Error code: {:}'.format(status))
            status = -1
        
        return status


    def enable_input_relay(self):
        """Closes the input relay, connecting the input voltage to the power
        circuit.

        On the hardware, the input voltage is also connected to the power
        circuit through the pre-charge resistors.

        Returns
        -------
        status : int
            Returns 0 if command was executed properly, -1 otherwise.
        
        """
        gpio = self.hwm.gpio_input_relay
        status = self.plat.cpu2_gpio(gpio, 1)

        if status != 0:
            print('\nError opening the input relay. Error code: {:}'.format(status))
            status = -1
        
        return status


    def disable_output_relay(self):
        """Opens the output relay, disconnecting the output load from the
        power circuit.

        Returns
        -------
        status : int
            Returns 0 if command was executed properly, -1 otherwise.
        
        """
        gpio = self.hwm.gpio_output_relay
        status = self.plat.cpu2_gpio(gpio, 0)

        if status != 0:
            print('\nError opening the output relay. Error code: {:}'.format(status))
            status = -1
        
        return status


    def enable_output_relay(self):
        """Closes the output relay, connecting the output load to the power
        circuit.

        Returns
        -------
        status : int
            Returns 0 if command was executed properly, -1 otherwise.
        
        """
        gpio = self.hwm.gpio_output_relay
        status = self.plat.cpu2_gpio(gpio, 1)

        if status != 0:
            print('\nError closing the output relay. Error code: {:}'.format(status))
            status = -1
        
        return status


    def set_control_mode(self, mode, params):
        """Sets control mode.

        Parameters
        ------
        mode : str
            Controller.

        params : dict
            A dictionary containing the controller parameters.

        Returns
        -------
        status : int
            Returns 0 if command was executed properly, -1 otherwise.
        
        """
        status = self.plat.cpu2_control_mode_set(mode, params)
        
        if status != 0:
            print('\nError setting the control mode. Error code: {:}'.format(status))
            status = -1
        
        return status


    def set_observer_mode(self, mode, params):
        """Sets observer mode.

        Parameters
        ------
        mode : str
            Observer.

        params : dict
            A dictionary containing observer parameters.

        Returns
        -------
        status : int
            Returns 0 if command was executed properly, -1 otherwise.
        
        """
        status = self.plat.cpu2_observer_mode_set(mode, params)
        
        if status != 0:
            print('\nError setting the observer mode. Error code: {:}'.format(status))
            status = -1
        
        return status

    
    def set_reference(self, ref):
        """Sets the reference for control modes other than open-loop.

        Parameters
        ----------
        ref : int
            Reference.

        Returns
        -------
        status : int
            Returns 0 if command was executed properly, -1 otherwise.
        
        """
        
        # Check for Right Input
        if type(ref) is not int and type(ref) is not float:
            raise TypeError('`ref` must be of `int` or `float` type.')

        if ref > self.hwm.vref_limit or ref < 0:
            raise ValueError('`ref` must be a value between 0 and ' + str(self.hwm.vref_limit) + '.')         
         
        # Get Reference from 0 -30 V  to 0 - 4095
        ref_adc = ref * (4095/self.hwm.vref_limit);
        ref_adc = round(ref_adc); #Getting Integer Value
        
        
        status = self.plat.cpu2_ref_set(ref_adc)
        
        if status != 0:
            print('\nError setting the reference. Error code: {:}'.format(status))
            status = -1
        
        return status
    
    
    def enable_pwm(self):
        """Enables PWM signal and starts control of the converter.

        The PWM is only enabled if the controller status is zero.

        Returns
        -------
        status : int
            Returns 0 if PWM was enabled, and -1 if an error occurred.
            
        """
        status = self.plat.cpu2_pwm_enable()

        if status != 0:
            print('\nError enabling PWM. Error code: {:}'.format(status))
            status = -1
        
        return status
    

    def disable_pwm(self):
        """Disables PWM signal and stops control of the converter.

        Returns
        -------
        status : int
            Returns 0 if PWM was disabled, and -1 if an error occurred.
            
        """
        status = self.plat.cpu2_pwm_disable()

        if status != 0:
            print('\nError disabling PWM. Error code: {:}'.format(status))
            status = -1
        
        return status


    def read_vin_buffer(self):
        """Reads the samples stored in the Vin ADC buffer.

        Returns
        -------
        data or status : np.array or int
            Returns the samples as a numpy array if the data was read
            successfully. Otherwise, -1 is returned.

        """
        adc = self.hwm.adc_vin

        data = self.plat.cpu1_adc_buffer_read(adc)

        if type(data) is int:
            print('Error reading Vin buffer. Error code: {:}'.format(data))
            return -1

        data = np.array(data)
    # Calculate back in Voltage value        
        data = data * self.hwm.vin_gain + self.hwm.vin_offset

        return data


    def read_vin_buck_buffer(self):
        """Reads the samples stored in the Vin_buck ADC buffer.

        Returns
        -------
        data or status : np.array or int
            Returns the samples as a numpy array if the data was read
            successfully. Otherwise, -1 is returned.

        """
        adc = self.hwm.adc_vin_buck

        data = self.plat.cpu1_adc_buffer_read(adc)

        if type(data) is int:
            print('Error reading Vin_buck buffer. Error code: {:}'.format(data))
            return -1

        data = np.array(data)
    # Calculate back in Voltage value        
        data = data * self.hwm.vin_buck_gain + self.hwm.vin_buck_offset

        return data


    def read_vout_buffer(self):
        """Reads the samples stored in the Vout ADC buffer.

        Returns
        -------
        data or status : np.array or int
            Returns the samples as a numpy array if the data was read
            successfully. Otherwise, -1 is returned.

        """
        adc = self.hwm.adc_vout

        data = self.plat.cpu1_adc_buffer_read(adc)

        if type(data) is int:
            print('Error reading Vin buffer. Error code: {:}'.format(data))
            return -1

        data = np.array(data)
    # Calculate back in Voltage value        
        data = data * self.hwm.vout_gain + self.hwm.vout_offset
        
        return data


    def read_vout_buck_buffer(self):
        """Reads the samples stored in the Vout_buck ADC buffer.

        Returns
        -------
        data or status : np.array or int
            Returns the samples as a numpy array if the data was read
            successfully. Otherwise, -1 is returned.

        """
        adc = self.hwm.adc_vout_buck

        data = self.plat.cpu1_adc_buffer_read(adc)

        if type(data) is int:
            print('Error reading Vout_buck buffer. Error code: {:}'.format(data))
            return -1

        data = np.array(data)
    # Calculate back in Voltage value        
        data = data * self.hwm.vout_buck_gain + self.hwm.vout_buck_offset
        
        return data


    def read_il_buffer(self):
        """Reads the samples stored in the IL ADC buffer.

        Returns
        -------
        data or status : np.array or int
            Returns the samples as a numpy array if the data was read
            successfully. Otherwise, -1 is returned.

        """
        adc = self.hwm.adc_il

        data = self.plat.cpu1_adc_buffer_read(adc)

        if type(data) is int:
            print('Error reading IL buffer. Error code: {:}'.format(data))
            return -1

        data = np.array(data)
    # Calculate back in Current (A) value     

        data = data * self.hwm.il_gain + self.hwm.il_offset

        return data


    def read_il_avg_buffer(self):
        """Reads the samples stored in the IL_avg ADC buffer.

        Returns
        -------
        data or status : np.array or int
            Returns the samples as a numpy array if the data was read
            successfully. Otherwise, -1 is returned.

        """
        adc = self.hwm.adc_il_avg

        data = self.plat.cpu1_adc_buffer_read(adc)

        if type(data) is int:
            print('Error reading IL_avg buffer. Error code: {:}'.format(data))
            return -1

        data = np.array(data)

    # Calculate back in Current (A) value     

        data = data * self.hwm.il_avg_gain + self.hwm.il_avg_offset
    
        return data


    def read_u_buffer(self):
        """Reads the samples stored in the `u` buffer.

        Returns
        -------
        data or status : np.array or int
            Returns the samples as a numpy array if the data was read
            successfully. Otherwise, -1 is returned.

        """
        adc = self.hwm.cpu2_buffer_u

        data = self.plat.cpu2_buffer_read(adc)

        if type(data) is int:
            print('Error reading `u` buffer. Error code: {:}'.format(data))
            return -1

        data = np.array(data)
        
        data = data * self.hwm.u_gain + self.hwm.u_offset
        return data


    def read_all_data(self):
        """Reads data from all ADCs and the control signal.

        Returns
        -------
        data or status : list or int
            Returns a list where each element is a numpy array, containing
            the measurements. Elements 0 and 1 are Vin and Vin_buck, elements
            2 and 3 are Vout and Vout_buck, elements 4 and 5 are IL and
            IL_avg and element 6 is the control signal. If an error occurs
            when reading the data, returns -1.
            
        """
        while 1:
            vin = self.read_vin_buffer()
            if type(vin) is not int: break
            print('Could not read Vin buffer... trying again.') 

        while 1:
            vin_buck = self.read_vin_buck_buffer()
            if type(vin_buck) is not int: break
            print('Could not read Vin_buck buffer... trying again.')

        while 1:
            vout = self.read_vout_buffer()
            if type(vout) is not int: break
            print('Could not read Vout buffer... trying again.')

        while 1:
            vout_buck = self.read_vout_buck_buffer()
            if type(vout_buck) is not int: break
            print('Could not read Vout_buck buffer... trying again.')
            
        while 1:
            il = self.read_il_buffer()
            if type(il) is not int: break
            print('Could not read IL buffer... trying again.')

        while 1:
            il_avg = self.read_il_avg_buffer()
            if type(il_avg) is not int: break
            print('Could not read IL_avg buffer... trying again.')            

        while 1:
            u = self.read_u_buffer()
            if type(u) is not int: break
            print('Could not read u buffer... trying again.')

        return [vin, vin_buck, vout, vout_buck, il, il_avg, u]


    def _set_trip_vin(self, trip):
        """Sets the trip value for Vin.

        If an ADC sample is higher than the tripping limit, the PWM signal is
        disabled on the hardware.

        Parameters
        ----------
        trip : int
            The tripping value.

        Returns
        -------
        status : int
            Returns 0 if the trip was set. Otherwise, returns -1.

        """
        adc = self.hwm.adc_vin
        
        #Trip into ADC-Value from Voltage
        trip = round(trip * self.hwm.adc_Resolution / self.hwm.vref_limit)

        status = self.plat.cpu2_trip_set(adc, trip)

        if status != 0:
            print('Error setting the tripping value for Vin. Error code: {:}'.format(status))
            return -1

        return 0


    def _read_trip_vin(self):
        """Reads the trip value of Vin.

        Returns
        -------
        status : int
            Returns the triping value, as a positive integer. If an error
            occurred, returns -1.
            
        """
        adc = self.hwm.adc_vin

        trip = self.plat.cpu2_trip_read(adc)

        if trip < 0:
            print('Error getting the tripping value for Vin. Error code: {:}'.format(trip))
            return -1

        return trip
    

    def _set_trip_vin_buck(self, trip):
        """Sets the trip value for Vin_buck.

        If an ADC sample is higher than the tripping limit, the PWM signal is
        disabled on the hardware.

        Parameters
        ----------
        trip : int
            The tripping value.

        Returns
        -------
        status : int
            Returns 0 if the trip was set. Otherwise, returns -1.

        """
        adc = self.hwm.adc_vin_buck
        
        #Trip into ADC-Value from Voltage
        trip = round(trip * self.hwm.adc_Resolution / self.hwm.vref_limit)
        
        status = self.plat.cpu2_trip_set(adc, trip)

        if status != 0:
            print('Error setting the tripping value for Vin_buck. Error code: {:}'.format(status))
            return -1

        return 0


    def _read_trip_vin_buck(self):
        """Reads the trip value of Vin_buck.

        Returns
        -------
        status : int
            Returns the triping value, as a positive integer. If an error
            occurred, returns -1.
            
        """
        adc = self.hwm.adc_vin_buck

        trip = self.plat.cpu2_trip_read(adc)

        if trip < 0:
            print('Error getting the tripping value for Vin_buck. Error code: {:}'.format(trip))
            return -1

        return trip


    def _set_trip_vout(self, trip):
        """Sets the trip value for Vout.

        If an ADC sample is higher than the tripping limit, the PWM signal is
        disabled on the hardware.

        Parameters
        ----------
        trip : int
            The tripping value.

        Returns
        -------
        status : int
            Returns 0 if the trip was set. Otherwise, returns -1.

        """
        adc = self.hwm.adc_vout       
        #Trip into ADC-Value from Voltage
        trip = round(trip * self.hwm.adc_Resolution / self.hwm.vref_limit)
        
        status = self.plat.cpu2_trip_set(adc, trip)

        if status != 0:
            print('Error setting the tripping value for Vout. Error code: {:}'.format(status))
            return -1

        return 0


    def _read_trip_vout(self):
        """Reads the trip value of Vout.

        Returns
        -------
        status : int
            Returns the triping value, as a positive integer. If an error
            occurred, returns -1.
            
        """
        adc = self.hwm.adc_vout

        trip = self.plat.cpu2_trip_read(adc)

        if trip < 0:
            print('Error getting the tripping value for Vout. Error code: {:}'.format(trip))
            return -1

        return trip


    def _set_trip_vout_buck(self, trip):
        """Sets the trip value for Vout_buck.

        If an ADC sample is higher than the tripping limit, the PWM signal is
        disabled on the hardware.

        Parameters
        ----------
        trip : int
            The tripping value.

        Returns
        -------
        status : int
            Returns 0 if the trip was set. Otherwise, returns -1.

        """
        adc = self.hwm.adc_vout_buck
        #Trip into ADC-Value from Voltage
        trip = round(trip * self.hwm.adc_Resolution / self.hwm.vref_limit)
        
        status = self.plat.cpu2_trip_set(adc, trip)

        if status != 0:
            print('Error setting the tripping value for Vout_buck. Error code: {:}'.format(status))
            return -1

        return 0


    def _read_trip_vout_buck(self):
        """Reads the trip value of Vout_buck.

        Returns
        -------
        status : int
            Returns the triping value, as a positive integer. If an error
            occurred, returns -1.
            
        """
        adc = self.hwm.adc_vout_buck

        trip = self.plat.cpu2_trip_read(adc)

        if trip < 0:
            print('Error getting the tripping value for Vout_buck. Error code: {:}'.format(trip))
            return -1

        return trip


    def _set_trip_il(self, trip):
        """Sets the trip value for IL.

        If an ADC sample is higher than the tripping limit, the PWM signal is
        disabled on the hardware.

        Parameters
        ----------
        trip : int
            The tripping value.

        Returns
        -------
        status : int
            Returns 0 if the trip was set. Otherwise, returns -1.

        """
        adc = self.hwm.adc_il      
        trip = round((trip-self.hwm.il_sensor_offset/self.hwm.il_sensor_sensitiviy)/(self.hwm.il_adc_voltage_gain/(self.hwm.il_resistor_gain*self.hwm.il_sensor_sensitiviy)))

        status = self.plat.cpu2_trip_set(adc, trip)

        if status != 0:
            print('Error setting the tripping value for IL. Error code: {:}'.format(status))
            return -1

        return 0


    def _read_trip_il(self):
        """Reads the trip value of IL.

        Returns
        -------
        status : int
            Returns the triping value, as a positive integer. If an error
            occurred, returns -1.
            
        """
        adc = self.hwm.adc_il

        trip = self.plat.cpu2_trip_read(adc)

        if trip < 0:
            print('Error getting the tripping value for IL. Error code: {:}'.format(trip))
            return -1

        return trip


    def _set_trip_il_avg(self, trip):
        """Sets the trip value for IL_avg.

        If an ADC sample is higher than the tripping limit, the PWM signal is
        disabled on the hardware.

        Parameters
        ----------
        trip : int
            The tripping value.

        Returns
        -------
        status : int
            Returns 0 if the trip was set. Otherwise, returns -1.

        """
        adc = self.hwm.adc_il_avg
        trip = round((trip-self.hwm.il_sensor_offset/self.hwm.il_sensor_sensitiviy)/(self.hwm.il_adc_voltage_gain/(self.hwm.il_resistor_gain*self.hwm.il_sensor_sensitiviy)))
        
        status = self.plat.cpu2_trip_set(adc, trip)

        if status != 0:
            print('Error setting the tripping value for IL_avg. Error code: {:}'.format(status))
            return -1

        return 0


    def _read_trip_il_avg(self):
        """Reads the trip value of IL_avg.

        Returns
        -------
        status : int
            Returns the triping value, as a positive integer. If an error
            occurred, returns -1.
            
        """
        adc = self.hwm.adc_il_avg

        trip = self.plat.cpu2_trip_read(adc)

        if trip < 0:
            print('Error getting the tripping value for IL_avg. Error code: {:}'.format(trip))
            return -1

        return trip


    def experiment(self, ref, control, params, obs=None, obs_params=None):
        """Runs an experiment.

        The experiment consists of setting control mode, closing the
        input/output relays, enabling the PWM for a certain amount of time,
        disabling the PWM and opening the input/output relays.

        Data from the experiment is then returned.

        Returns
        -------
        data or status : list or int
            If no errors occurred during the experiment, data from the ADCs
            is returned. Otherwise, returns -1.
            
        """
        status = self.set_reference(ref)
        
        status = self.set_control_mode(control, params)
        if status != 0:
            print('Could not set control mode. Aborting experiment.')
            return -1

        if obs is not None:
            status = self.set_observer_mode(obs, obs_params)
            if status != 0:
                print('Could not set control mode. Aborting experiment.')
                return -1
        
        status = self.enable_input_relay()
        if status != 0:
            print('Could not close input relay. Aborting experiment.')
            return -1

        status = self.enable_output_relay()
        if status != 0:
            print('Could not close input relay. Aborting experiment.')
            while 1:
                status = self.disable_input_relay()
                if status == 0: break
            while 1:
                status = self.disable_output_relay()
                if status == 0: break
            return -1

        time.sleep(2)

        status = self.enable_pwm()
        if status != 0:
            print('Could not enable PWM. Aborting experiment.')
            while 1:
                status = self.disable_input_relay()
                if status == 0: break
            while 1:
                status = self.disable_output_relay()
                if status == 0: break
            return -1

        time.sleep(3)

        while 1:
            status = self.disable_pwm()
            status = self.disable_pwm()
            if status == 0: break
            print('Could not disable PWM signal... trying again.')

        time.sleep(2)
        
        while 1:
            status = self.disable_input_relay()
            if status == 0: break
            print('Could not open input relay... trying again.')

        while 1:
            status = self.disable_output_relay()
            if status == 0: break
            print('Could not open output relay... trying again.')

        while 1:
            vin = self.read_vin_buffer()
            if type(vin) is not int: break
            print('Could not read Vin buffer... trying again.')

        while 1:
            vin_buck = self.read_vin_buck_buffer()
            if type(vin_buck) is not int: break
            print('Could not read Vin_buck buffer... trying again.')

        while 1:
            vout = self.read_vout_buffer()
            if type(vout) is not int: break
            print('Could not read Vout buffer... trying again.')

        while 1:
            vout_buck = self.read_vout_buck_buffer()
            if type(vout_buck) is not int: break
            print('Could not read Vout_buck buffer... trying again.')
            
        while 1:
            il = self.read_il_buffer()
            if type(il) is not int: break
            print('Could not read IL buffer... trying again.')

        while 1:
            il_avg = self.read_il_avg_buffer()
            if type(il_avg) is not int: break
            print('Could not read IL_avg buffer... trying again.')            

        while 1:
            u = self.read_u_buffer()
            if type(u) is not int: break
            print('Could not read u buffer... trying again.')
            
        #Creating time Vector: 
        # t in mili seconds
        SampleList = list(range(0,len(vout)));
        t = [element *self.hwm.sample_time*1000 for element in SampleList];
        t = np.array(t) # t in mili seconds
        


        data = [t, vin, vin_buck, vout, vout_buck, il, il_avg, u]

        return data
            
