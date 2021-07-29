import cpeplat as cpe
import numpy as np

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
        self.vin_trip = 3000
        self.vin_buck_trip = 3000

        self.vout_trip = 3000
        self.vout_buck_trip = 3000

        self.il_trip = 3200
        self.il_avg_trip = 3200

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
        status = plat.cpu2_trip_set(hwm.adc_vin, hw_default.vin_trip)
        status |= plat.cpu2_trip_set(hwm.adc_vin_buck, hw_default.vin_buck_trip)
        
        status |= plat.cpu2_trip_set(hwm.adc_vout, hw_default.vout_trip)
        status |= plat.cpu2_trip_set(hwm.adc_vout_buck, hw_default.vout_buck_trip)
        
        status |= plat.cpu2_trip_set(hwm.adc_il, hw_default.il_trip)
        status |= plat.cpu2_trip_set(hwm.adc_il_avg, hw_default.il_avg_trip)

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


    def status(self):
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

        
    def status_clear(self):
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
    

    def open_input_relay(self):
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


    def close_input_relay(self):
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


    def open_output_relay(self):
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


    def close_output_relay(self):
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


    def control_mode(self, mode, params):
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
        status = self.plat.cpu2_ref_set(ref)
        
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

        return data
