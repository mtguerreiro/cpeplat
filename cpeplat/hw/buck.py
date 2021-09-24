import cpeplat as cpe
import numpy as np
import time
import cpeplat.result as res


class BuckHWM:
    """A class to hold hardware mapping data for the buck platform.

    Here, we list:

    - Input and output GPIO pins
    - ADCs' indexes for measurements
    - Gains of voltage and current sensors, as well as control signal
    - Additional buffers for CPU2

    Parameters
    ----------
    f_pwm : int, float
        PWM frequency, which defines the gain for the control signal. By
        default, it is 200 kHz.

    Attributes
    ----------
    adc_vin : int
        ADC index corresponding to the measurement of V_in. By default,
        it is 2 and should reflect the actual index of the C-code.

    adc_vin_buck : int
        ADC index corresponding to the measurement of V_in_buck. By default,
        it is 1 and should reflect the actual index of the C-code.

    adc_vout : int
        ADC index corresponding to the measurement of V_out. By default,
        it is 3 and should reflect the actual index of the C-code.

    adc_vout_buck : int
        ADC index corresponding to the measurement of V_out_buck. By default,
        it is 5 and should reflect the actual index of the C-code.

    adc_il : int
        ADC index corresponding to the measurement of IL. By default,
        it is 0 and should reflect the actual index of the C-code.

    adc_il_avg : int
        ADC index corresponding to the measurement of IL_avg. By default,
        it is 4 and should reflect the actual index of the C-code.

    cpu2_buffer_u : int
        CPU2 buffer index for the control signal. By default, it is 0 and
        should reflect the actual index of the C-code.

    cpu2_buffer_1 : int
        CPU2 buffer index for an additional CPU2 buffer. By default, it is 1.

    cpu2_buffer_2 : int
        CPU2 buffer index for an additional CPU2 buffer. By default, it is 2.

    cpu2_buffer_3 : int
        CPU2 buffer index for an additional CPU2 buffer. By default, it is 3.

    gpio_input_relay : int
        GPIO of the input relay. By default, it is 8 and should reflect the
        actual pin to which the relay's driver is connected to.

    gpio_output_relay : int
        GPIO of the output relay. By default, it is 9 and should reflect the
        actual pin to which the relay's driver is connected to.

    gpio_fault_enable : int
        GPIO of the fault switch. By default, it is 10 and should reflect the
        actual pin to which the switch's driver is connected to.

    adc_resolution : int
        Range of the ADC. For instance, for 12 bits, the range is 0-4095. By
        default, it is 4095.

    adc_voltage : float, int
        Reference voltage of the ADC for conversion. By default, it is 3.

    f_pwm : float, int
        PWM frequency. By default, it is 200 kHz. Changing this value will not
        change the switching frequency on the hardware.

    sample_time : float, int
        The inverse of the PWM frequency.

    vin_adc_gain : float, int
        Gain for the ADC of the V_in measurement. Usually, the ADC gain is
        simply (adc_voltage / adc_resolution). This is the default case.

    vin_sensor_gain : float, int
        Gain of the voltage sensor. This information comes from the hardware.
        By default, it is (16.01 / 1.5822). This value will not change the
        gain used by the control algorithm on the hardware, and it is only
        used here to convert the raw ADC measurements to voltage.

    vin_gain : float, int
        Total gain of the V_in measurement. It is the ADC gain times the
        sensor gain. This value will not change the gain used by the control
        algorithm on the hardware, and it is only used here to convert the
        raw ADC measurements to voltage.

    vin_offset : float, int
        Total offset of the V_in measurement. By default, it is 0.

    vin_buck_adc_gain : float, int
        See :attr:`.vin_adc_gain`.

    vin_buck_sensor_gain : float, int
        See :attr:`.vin_sensor_gain`.

    vin_buck_gain : float, int
        See :attr:`.vin_gain`.

    vin_buck_offset : float, int
        See :attr:`.vin_offset`.

    vout_adc_gain : float, int
        See :attr:`.vin_adc_gain`.

    vout_sensor_gain : float, int
        See :attr:`.vin_sensor_gain`.

    vout_gain : float, int
        See :attr:`.vin_gain`.

    vout_offset : float, int
        See :attr:`.vin_offset`.

    vout_buck_adc_gain : float, int
        See :attr:`.vin_adc_gain`.

    vout_buck_sensor_gain : float, int
        See :attr:`.vin_sensor_gain`.

    vout_buck_gain : float, int
        See :attr:`.vin_gain`.

    vout_buck_offset : float, int
        See :attr:`.vin_offset`.

    il_adc_gain : float, int
        See :attr:`.vin_adc_gain`.

    il_sensor_gain : float, int
        See :attr:`.vin_sensor_gain`.

    il_gain : float, int
        See :attr:`.vin_gain`.

    il_offset : float, int
        See :attr:`.vin_offset`.

    il_avg_adc_gain : float, int
        See :attr:`.vin_adc_gain`.

    il_avg_sensor_gain : float, int
        See :attr:`.vin_sensor_gain`.

    il_avg_gain : float, int
        See :attr:`.vin_gain`.

    il_avg_offset : float, int
        See :attr:`.vin_offset`.

    u_gain : float, int
        Gain for the control signal. The hardware saves the control signal as
        a timer value. This gains converts the timer value to the 0-1 range.

    u_offset : float, int
        Offset for the control signal. By default, it is 0.

    ref_adc_gain : float, int
        See :attr:`.vin_adc_gain`.

    ref_sensor_gain : float, int
        See :attr:`.vin_sensor_gain`.

    ref_gain : float, int
        See :attr:`.vin_gain`.

    """
    def __init__(self, f_pwm=200e3):
        self.adc_vin = 2
        self.adc_vin_buck = 1
        self.adc_vout = 3
        self.adc_vout_buck = 5
        self.adc_il = 0
        self.adc_il_avg = 4
        
        self.cpu2_buffer_u = 0
        self.cpu2_buffer_1 = 1
        self.cpu2_buffer_2 = 2
        self.cpu2_buffer_3 = 3

        self.gpio_input_relay = 8
        self.gpio_output_relay = 9
        self.gpio_fault_enable = 10
        
        self.adc_resolution = 4095
        self.adc_voltage = 3

        self.f_pwm = f_pwm
        self.sample_time = 1 / f_pwm

        # Vin measurements
        self.vin_adc_gain = self.adc_voltage / self.adc_resolution
        self.vin_sensor_gain = (16.01 / 1.5822)
        self.vin_gain = self.vin_adc_gain * self.vin_sensor_gain
        self.vin_offset = 0

        # Vin_buck measurements
        self.vin_buck_adc_gain = self.adc_voltage / self.adc_resolution
        self.vin_buck_sensor_gain = (16.01 / 1.5838)
        self.vin_buck_gain = self.vin_buck_adc_gain * self.vin_buck_sensor_gain
        self.vin_buck_offset = 0

        # Vout measurements
        self.vout_adc_gain = self.adc_voltage / self.adc_resolution
        self.vout_sensor_gain = (8.07 / 0.7970)
        self.vout_gain = self.vout_adc_gain * self.vout_sensor_gain
        self.vout_offset = 0

        # Vout_buck measurements
        self.vout_buck_adc_gain = self.adc_voltage / self.adc_resolution
        self.vout_buck_sensor_gain = (8.07 / 0.7970)
        self.vout_buck_gain = self.vout_buck_adc_gain * self.vout_buck_sensor_gain
        self.vout_buck_offset = 0

        # IL measurements
        self.il_adc_gain = self.adc_voltage / self.adc_resolution
        self.il_sensor_gain = (5.9 / 3.9) / 50e-3
        self.il_gain = self.il_adc_gain * self.il_sensor_gain
        self.il_offset = -(2.49 / 50e-3 + 2 * 0.1958)

        # IL_avg measurements
        self.il_avg_adc_gain = self.adc_voltage / self.adc_resolution
        self.il_avg_sensor_gain = (5.9 / 3.9) / 50e-3
        self.il_avg_gain = self.il_avg_adc_gain * self.il_avg_sensor_gain
        self.il_avg_offset = -(2.49 / 50e-3)
        
        # Control signal
        self.u_gain = 1 / ((100e6 / f_pwm) - 1)
        self.u_offset = 0

        # Reference signal
        self.ref_adc_gain = self.adc_voltage / self.adc_resolution
        self.ref_sensor_gain = 10
        self.ref_gain = self.ref_adc_gain * self.ref_sensor_gain


class BuckHWDefaultSettings:
    """A class to hold default hardware settings for the buck platform.

    Attributes
    ----------
    vin_buffer_size : int
        Default size of V_in buffer.

    vin_buck_buffer_size : int
        Default size of V_in_buck buffer.

    vout_buffer_size : int
        Default size of V_out buffer.

    vout_buck_buffer_size : int
        Default size of V_out_buck buffer.

    il_buffer_size : int
        Default size of IL buffer.

    il_avg_buffer_size : int
        Default size of IL_avg buffer.

    u_buffer_size : int
        Default size of control buffer.

    vin_trip : int, float
        Default tripping value for V_in.

    vin_buck_trip : int, float
        Default tripping value for V_in_buck.

    vout_trip : int, float
        Default tripping value for V_out.

    vout_buck_trip : int, float
        Default tripping value for V_out_buck.

    il_trip : int, float
        Default tripping value for V_in.

    il_avg_buck_trip : int, float
        Default tripping value for V_in_buck.

    cpu1_blink : int
        Blinking rate for CPU1's LED.

    cpu2_blink : int
        Blinking rate for CPU1's LED.
        
    """
    def __init__(self):

        # Buffers
        self.vin_buffer_size = 750
        self.vin_buck_buffer_size = 750

        self.il_buffer_size = 750
        self.vout_buffer_size = 750

        self.il_avg_buffer_size = 750
        self.vout_buck_buffer_size = 750

        self.u_buffer_size = 750

        # Tripping
        self.vin_trip = 22
        self.vin_buck_trip = 22

        self.vout_trip = 22
        self.vout_buck_trip = 22

        self.il_trip = 25
        self.il_avg_trip = 25

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

    The ADCs measuring these signals are (this comes from hardware arrangement
    and the C-code):
    
    - ADC_A_SOC0 (ADC_A5): IL
    - ADC_A_SOC1 (ADC_A4): V_in_buck
    - ADC_A_SOC2 (ADC_A1): V_in
    - ADC_B_SOC0 (ADC_B4): V_out
    - ADC_B_SOC1 (ADC_B5): IL_avg
    - ADC_C_SOC0 (ADC_C4): V_out_buck
    
    From the :class:`cpe.interface.Interface` class, we can check which ADC
    index corresponds to each ADC. Currently, they are:
    
    - ADC 0: IL
    - ADC 1: V_in_buck
    - ADC 2: V_in
    - ADC 3: V_out
    - ADC 4: IL_avg
    - ADC 5: V_out_buck
    
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

    f_pwm : float
        Switching frequency on the platform. This value is not sent to the
        hardware.

    Attributes
    ----------
    plat : :class:`cpeplat.interface.Interface`
        Interface with the platform.

    hwm : :class:`cpeplat.hw.BuckHWM`
        Hardware mapping and constants.

    hw_default : :class:`cpeplat.hw.BuckHWDefaultSettings`
        Default controller settings.

    
    Raises
    ------
    ValueError
        If setting any of the tripping values fail.
        
    """
    def __init__(self, com, baud=115200, to=0.2, f_pwm=200e3):

        plat = cpe.interface.Interface(com, baud, to)
        self.plat = plat
        
        hwm = BuckHWM(f_pwm)
        self.hwm = hwm

        hw_default = BuckHWDefaultSettings()
        self.hw_default = hw_default

        # Initializes ADC/CPU2 buffers
        while plat.cpu1_adc_buffer_set(hwm.adc_vin, hw_default.vin_buffer_size) != 0:
            print('Error setting CPU1 buffer. Trying again...')

        while plat.cpu1_adc_buffer_set(hwm.adc_vin_buck, hw_default.vin_buck_buffer_size) != 0:
            print('Error setting CPU1 buffer. Trying again...')
        
        while plat.cpu1_adc_buffer_set(hwm.adc_vout, hw_default.vout_buffer_size) != 0:
            print('Error setting CPU1 buffer. Trying again...')
        
        while plat.cpu1_adc_buffer_set(hwm.adc_vout_buck, hw_default.vout_buck_buffer_size) != 0:
            print('Error setting CPU1 buffer. Trying again...')
        
        while plat.cpu1_adc_buffer_set(hwm.adc_il, hw_default.il_buffer_size) != 0:
            print('Error setting CPU1 buffer. Trying again...')

        while plat.cpu1_adc_buffer_set(hwm.adc_il_avg, hw_default.il_avg_buffer_size) != 0:
            print('Error setting CPU1 buffer. Trying again...')

        while plat.cpu2_buffer_set(hwm.cpu2_buffer_u, hw_default.u_buffer_size) != 0:
            print('Error setting CPU2 buffer. Trying again...')

        while plat.cpu2_buffer_set(hwm.cpu2_buffer_1, hw_default.u_buffer_size) != 0:
            print('Error setting CPU2 buffer. Trying again...')

        while plat.cpu2_buffer_set(hwm.cpu2_buffer_2, 2 * hw_default.u_buffer_size) != 0:
            print('Error setting CPU2 buffer. Trying again...')

        while plat.cpu2_buffer_set(hwm.cpu2_buffer_3, 2 * hw_default.u_buffer_size) != 0:
            print('Error setting CPU2 buffer. Trying again...')
                
        # Sets and enable tripping for all ADCs
        while self._set_trip_vin(hw_default.vin_trip) != 0:
            print('Error setting Vin trip. Trying again...')
        while plat.cpu2_trip_enable(hwm.adc_vin) != 0:
            print('Error enabling Vin trip. Trying again...')

        while self._set_trip_vin_buck(hw_default.vin_buck_trip) != 0:
            print('Error setting Vin_buck trip. Trying again...')
        while plat.cpu2_trip_enable(hwm.adc_vin_buck) != 0:
            print('Error enabling Vin_buck trip. Trying again...')

        while self._set_trip_vout(hw_default.vout_trip) != 0:
            print('Error setting Vout trip. Trying again...')
        while plat.cpu2_trip_enable(hwm.adc_vout) != 0:
            print('Error enabling Vout trip. Trying again...')

        while self._set_trip_vout_buck(hw_default.vout_buck_trip) != 0:
            print('Error setting Vout_buck trip. Trying again...')
        while plat.cpu2_trip_enable(hwm.adc_vout_buck) != 0:
            print('Error enabling Vout_buck trip. Trying again...')

        while self._set_trip_il(hw_default.il_trip) != 0:
            print('Error setting IL trip. Trying again...')
        while plat.cpu2_trip_enable(hwm.adc_il) != 0:
            print('Error enabling IL trip. Trying again...')
            
        while self._set_trip_il_avg(hw_default.il_avg_trip) != 0:
            print('Error setting IL_avg trip. Trying again...')
        while plat.cpu2_trip_enable(hwm.adc_il_avg) != 0:
            print('Error enabling IL_avg trip. Trying again...')

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


    def enable_observer(self):
        """Enables the observer.

        Returns
        -------
        status : int
            Returns 0 if command was executed properly, -1 otherwise.
        
        """
        status = self.plat.cpu2_observer_enable()
        
        if status != 0:
            print('\nError enabling the observer. Error code: {:}'.format(status))
            status = -1
        
        return status


    def disable_observer(self):
        """Disables the observer.

        Returns
        -------
        status : int
            Returns 0 if command was executed properly, -1 otherwise.
        
        """
        status = self.plat.cpu2_observer_disable()
        
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

        ref_limit = self.hwm.ref_gain * self.hwm.adc_resolution
        if ref > ref_limit or ref < 0:
            raise ValueError('`ref` must be a value between 0 and ' + str(ref_limit) + '.')         
        
        ref_adc = round(ref / self.hwm.ref_gain)
        
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

        #Adding Time Signal, in MilliSeconds
        SampleList = list(range(0,len(vout)));
        t = [element * self.hwm.sample_time * 1000 for element in SampleList];
        t = np.array(t) # t in mili seconds
        
        return [t, vin, vin_buck, vout, vout_buck, il, il_avg, u]


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
        trip = round(trip / self.hwm.vin_gain)

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

        trip = trip * self.hwm.vin_gain

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
        trip = round(trip / self.hwm.vin_buck_gain)
        
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

        trip = trip * self.hwm.vin_buck_gain

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
        trip = round(trip / self.hwm.vout_gain)
        
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

        trip = trip * self.hwm.vout_gain

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
        trip = round(trip / self.hwm.vout_buck_gain)
        
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

        trip = trip * self.hwm.vout_buck_gain

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
        
        trip = round((trip - self.hwm.il_offset) / self.hwm.il_gain)

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

        trip = trip * self.hwm.il_gain + self.hwm.il_offset

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
        
        trip = round((trip - self.hwm.il_avg_offset) / self.hwm.il_avg_gain)
        
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

        trip = trip * self.hwm.il_avg_gain + self.hwm.il_avg_offset

        return trip


    def experiment(self, ref, control, params={}, obs=None, obs_params={}):
        """Runs an experiment.

        The experiment consists of setting control mode, closing the
        input/output relays, enabling the PWM for a certain amount of time,
        disabling the PWM and opening the input/output relays.

        Data from the experiment is then returned.

        Parameters
        ----------
        ref : float, int
            Reference.

        control : str
            Control mode.

        params : dict
            Controller parameters.

        obs : str, NoneType
            Observer mode. If `None`, observer is not set. By default, it is
            `None`.

        obs_params : dict
            Observer parameters.

        Returns
        -------
        data or status : list or int
            If no errors occurred during the experiment, data from the ADCs
            is returned. Otherwise, returns -1.
            
        """
        status = self.set_reference(ref)
        if status != 0:
            print('Could not set reference. Aborting experiment.')
            return -1            
        
        status = self.set_control_mode(control, params)
        if status != 0:
            print('Could not set control mode. Aborting experiment.')
            return -1

        if obs is not None:
            status = self.set_observer_mode(obs, obs_params)
            if status != 0:
                print('Could not set observer mode. Aborting experiment.')
                return -1
            status = self.enable_observer()
            if status != 0:
                print('Could not enable observer. Aborting experiment.')
                return -1
        else:
            status = self.disable_observer()
            if status != 0:
                print('Could not disable observer. Aborting experiment.')
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
            
        # Creates time vector (mili seconds)
        t = self.hwm.sample_time * np.arange(vin.shape[0]) / 1e-3

        data = [t, vin, vin_buck, vout, vout_buck, il, il_avg, u]

        return data
    
            
    def experiment_tuning(self, ref, control, params, obs=None, obs_params=None):
        """Runs several experiment in a row with a set Control and different 
        Parameter-Sets which were given in a list.

        The experiment consists of setting control mode, closing the
        input/output relays, enabling the PWM for a certain amount of time,
        disabling the PWM and opening the input/output relays.

        Data from all the experiments is then returned.
        So that the different Parameter-Sets can be compared.

        Returns
        -------
        data or status : list or int
            If no errors occurred during the experiment, data from the ADCs
            is returned. Otherwise, returns -1.
        
        Raises
        ------
        NameError:
            If `mode` is not of Type ('pid', 'ol' or 'sfb'.)

        TypeError
            If `params` is not of `dict` type.
            
        """
        control_types_tuning = ['ol','pid','sfb']
        #Error Checking: Not supported Control                
        if control not in control_types_tuning:
            if control == 'matlab':
                raise NameError('Tuning of External Matlab Control currently not supported!')
            else:
                raise NameError('Unknwon Control Mode!')
                
        #Starting Tuning Experiment      
        data = []
        leg = []
        for para_set in params:
            data.append(self.experiment(ref,control,para_set, obs, obs_params))
            
        if control == 'ol':
            title = 'Compare Different OpenLoop Tunings'
            for para_set in params:
                leg.append('u = ' + str(round(para_set['u'],3)))
        elif control == 'pid':
            title = 'Compare Different PID Tunings'
            counter = 0
            for para_set in params:
                leg.append('PID '+ str(counter+1))
                counter = counter + 1
        elif control == 'sfb':
            title = 'Compare Different State Feedback Controller'
            counter = 0
            for para_set in params:
                leg.append('SFB '+ str(counter+1))
                counter = counter + 1
                    
                    # Plotting Plot of Different Tunings
        res.plot_compare_all(data, leg = leg, title = title)
                                       
        return data


    def experiment_compare(self, ref, controllers, params, leg = None, title = None, obs=None, obs_params=None):
        """Runs several experiment in a row with different Controllers
        and compares the result

        The experiment consists of setting control mode, closing the
        input/output relays, enabling the PWM for a certain amount of time,
        disabling the PWM and opening the input/output relays.

        Data from all the experiments is then returned.
        So that the different Parameter-Sets can be compared.

        Returns
        -------
        data or status : list or int
            If no errors occurred during the experiment, data from the ADCs
            is returned. Otherwise, returns -1.
        
        Raises
        ------
        NameError:
            If `mode` is not of Type ('pid', 'ol' or 'sfb'.)

        TypeError
            If `params` is not of `dict` type.
            
        """
        # Checking for valid Control Type
        control_counter = 0
        data = []
        control_types_all = ['ol','matlab','pid','ol','sfb']
        
        for control in controllers:
        #Error Checking: Not supported Control                
            if control not in control_types_all:
                if control == 'matlab':
                    raise NameError('Unknwon Control Mode!')
                
        #Starting Tuning Experiment      
            data.append(self.experiment(ref, control, params[control_counter], obs, obs_params))
            control_counter = control_counter + 1
        
        if leg is None:
            leg = controllers

                   
        # Plotting Plot of Different Tunings
        res.plot_compare_all(data, leg = leg, title = title)
                                       
        return data            


    def experiment_disturbance(self, ref, control, params={}, obs=None, obs_params={}, event=False, event_params={}):
        """Runs an experiment, with disturbance.

        This is an experimental function. Be aware.

        The following is executed in sequence:

        - Reference, controller and observer are set.
        - Event is set.
        - Input relay is closed.
        - PWM is enabled.
        - Stalls for a certain amount of time.
        - PWM is disabled and relays are opened.

        Notice that the output relay is not closed. The output relay is
        expected to be closed/opened during the event, which happens on the
        controller.

        Parameters
        ----------
        ref : float, int
            Reference.

        control : str
            Control mode.

        params : dict
            Controller parameters.

        obs : str, NoneType
            Observer mode. If `None`, observer is not set. By default, it is
            `None`.

        obs_params : dict
            Observer parameters.

        event : bool
            Enables/disables the event. By default, it is `False`.

        event_params : dict            
            Event parameters.

        Returns
        -------
        data or status : list or int
            If no errors occurred during the experiment, data from the ADCs
            is returned. Otherwise, returns -1.
            
        """
        status = self.set_reference(ref)
        if status != 0:
            print('Could not set reference. Aborting experiment.')
            return -1
        
        status = self.set_control_mode(control, params)
        if status != 0:
            print('Could not set control mode. Aborting experiment.')
            return -1

        if obs is not None:
            status = self.set_observer_mode(obs, obs_params)
            if status != 0:
                print('Could not set observer mode. Aborting experiment.')
                return -1
            status = self.enable_observer()
            if status != 0:
                print('Could not enable observer. Aborting experiment.')
                return -1
        else:
            status = self.disable_observer()
            if status != 0:
                print('Could not disable observer. Aborting experiment.')
                return -1
            
        if event is True:
            gpio = event_params['gpio']
            start = event_params['start']
            stop = event_params['end']
            status = self.plat.cpu2_event_set(gpio, start, stop)
            if status != 0:
                print('Could not set control mode. Aborting experiment.')
                return -1
        
        status = self.enable_input_relay()
        if status != 0:
            print('Could not close input relay. Aborting experiment.')
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

        while 1:
            niter = np.array(self.plat.cpu2_buffer_read(1))
            if type(niter) is not int: break
            print('Could not read niter buffer... trying again.')

        while 1:
            il_hat = np.array(self.plat.cpu2_buffer_read_float(2))
            if type(il_hat) is not int: break
            print('Could not read il_hat buffer... trying again.')

        while 1:
            vc_hat = np.array(self.plat.cpu2_buffer_read_float(3))
            if type(vc_hat) is not int: break
            print('Could not read vc_hat buffer... trying again.')
            
        # Creates time vector (mili seconds)
        t = self.hwm.sample_time * np.arange(vin.shape[0]) / 1e-3

        data = [t, vin, vin_buck, vout, vout_buck, il, il_avg, u, niter, il_hat, vc_hat]

        return data
