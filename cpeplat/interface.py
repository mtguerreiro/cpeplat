import serialp
import time
import struct

"""
"""

class Commands:
    """Just a list with the commands accepted by the platform.
    """
    def __init__(self):
        self.cpu1_blink = 0x01
        self.cpu2_status = 0x02
        self.cpu2_status_clear = 0x03
        self.cpu2_blink = 0x04
        self.cpu2_gpio = 0x05
        self.cpu2_pwm_enable = 0x06
        self.cpu2_pwm_disable = 0x07
        self.cpu1_adc_buffer_set = 0x08
        self.cpu1_adc_buffer_read = 0x09
        self.cpu2_buffer_set = 0x0A
        self.cpu2_buffer_read = 0x0B
        self.cpu2_control_mode_set = 0x0C
        self.cpu2_control_mode_read = 0x0D
        self.cpu2_ref_set = 0x0E
        self.cpu2_ref_read = 0x0F
        self.cpu2_trip_set = 0x10
        self.cpu2_trip_enable = 0x11
        self.cpu2_trip_disable = 0x12
        self.cpu2_trip_read = 0x13
        

class Interface:
    """A class to provide an interface to the C2000-based platform.

    ADC mapping
    -----------

    Functions concerning the ADCs index each individual ADC with a number
    between 0 and `N`, where `N` is the maximum number of ADCs. The ADCs
    indexes are mapped to the physical ADCs as follows:

    - ADC 0: ADCIN_A1
    - ADC 1: ADCIN_A4
    - ADC 2: ADCIN_A5
    - ADC 3: ADCIN_B4
    - ADC 4: ADCIN_B5
    - ADC 5: ADCIN_C4

    This mapping is valid for all ADC-related settings. For instance, setting
    the tripping limit for ADC 2 will set the tripping limit for ADC_A5,
    reading the buffer for ADC 4 will read the buffer of ADC_B5, and so on.

    CPU2 buffer
    -----------

    The data stored in CPU2's buffer depends on each application. Usually,
    buffer 0 will hold the control signal. Ultimately, it is up to the user to
    set the buffers and interpret the data.

    GPIOs
    -----

    Currently, the interface can set the level of any GPIO. However,
    initialization of the corresponding pin must be done in firmware. The
    current interface does not provide any GPIO initialization.
    

    Parameters
    ----------
    com : str
        COM port.

    baud : int
        Baudrate used for communication. By default, it is 9600 bps.

    timeout : int, float
        Communication time-out, in seconds. By default, it is 0.2 s.
        
    """
    def __init__(self, com, baud=115200, to=0.2):
        self.ser = serialp.serial.Serial(com, baud, to)
        self.cmd = Commands()
        

    def cpu1_blink(self, t=1000):
        """Changes the blinking period of CPU1.

        Parameters
        ----------
        t : int
            Period, in milliseconds. By default, it is 1000.

        Raises
        ------
        TypeError
            If `t` is not of `int` type.

        """
        if type(t) is not int:
            raise TypeError('`t` must be of int type.')
        
        t = serialp.conversions.u16_to_u8(t, msb=True)
        cmd = self.cmd.cpu1_blink

        self.ser.send(cmd, t)
        

    def cpu2_status(self):
        """Gets the status of CPU2.

        Returns
        -------
        status : int
            If status of CPU2 was received correctly, returns the status as a
            positive integer. Otherwise, returns a negative integer.
            
        """
        funcname = Interface.cpu2_status.__name__
        cmd = self.cmd.cpu2_status

        self.ser.send(cmd)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tFailed to communicate with CPU1.'.format(funcname))
            return -1
        
        if data[0] == 1:
            print('{:}|\tCommunicated with CPU1 but CPU2 is unresponsive. Status not read.'.format(funcname))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:], msb=True)
        print('{:}|\tCPU2 status: {:}'.format(funcname, status))

        return status


    def cpu2_status_clear(self):
        """Clears the status of CPU2.

        Returns
        -------
        status : int
            The status of CPU2, as a positive integer (should be zero if the
            command worked). Otherwise, returns a negative integer.
        
        """
        funcname = Interface.cpu2_status_clear.__name__
        cmd = self.cmd.cpu2_status_clear

        self.ser.send(cmd)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tFailed to communicate with CPU1.'.format(funcname))
            return -1

        if data[0] == 1:
            print('{:}|\tCommunicated with CPU1 but CPU2 is unresponsive. Status not cleared.'.format(funcname))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:], msb=True)        

        if status != 0:
            print('{:}|\tCommand failed. CPU2 status {:}.'.format(funcname, status))
            return status

        print('{:}|\tCPU2 status cleared. Status {:}.'.format(funcname, status))

        return 0
            
        
    def cpu2_blink(self, t=1000):
        """Changes the blinking period of CPU2.

        Parameters
        ----------
        t : int
            Period, in milliseconds. By default, it is 1000.

        Raises
        ------
        TypeError
            If `t` is not of `int` type.
            
        """
        if type(t) is not int:
            raise TypeError('`t` must be of int type.')
        
        t = serialp.conversions.u16_to_u8(t, msb=True)
        cmd = self.cmd.cpu2_blink

        self.ser.send(cmd, t)

    
    def cpu2_gpio(self, gpio, state):
        """Sets the level of a GPIO controlled by CPU2.

        The GPIO must have been properly initialized, i.e., set as output and
        ownership given by CPU1 to CPU2.
        
        Parameters
        ----------
        gpio : int
            GPIO to be set/reset.

        state : int
            State of GPIO. A value of 0 means the GPIO will be cleared, and a
            value of 1 or any other value means the GPIO will be set. 

        Returns
        -------
        status : int
            Status of command. If command was executed successfully, returns 0.
            Otherwise, returns a negative integer.
        
        Raises
        ------
        TypeError
            If either `gpio` or `state` is not of `int` type.
        
        """
        funcname = Interface.cpu2_gpio.__name__
        if type(gpio) is not int or type(state) is not int:
            raise TypeError('`gpio` and `state` must be of int type.')

        data = [gpio, state]
        cmd = self.cmd.cpu2_gpio

        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tGPIO {:}: failed to communicate with CPU1.'.format(funcname, gpio))
            return -1

        if data[0] == 1:
            print('{:}|\tGPIO {:}: communicated with CPU1 but CPU2 is unresponsive. GPIO not set.'.format(funcname, gpio))
            return -2

        state_rcvd = serialp.conversions.u8_to_u16(data[1:3], msb=True)
        gpio_rcvd = serialp.conversions.u8_to_u16(data[3:], msb=True)

        if gpio_rcvd != gpio:
            print('{:}|\tGPIO {:}: GPIO error. GPIO sent: {:}. GPIO received: {:}.'.format(funcname, gpio, gpio, gpio_rcvd))
            return -3

        if state_rcvd != state:
            print('{:}|\tGPIO {:}: Could no set state. State sent: {:}. State received: {:}.'.format(funcname, gpio, state, state_rcvd))
            return -4
        
        print('{:}|\tGPIO {:}: GPIO set. State: {:}.'.format(funcname, gpio_rcvd, state_rcvd))

        return 0


    def cpu2_pwm_enable(self):
        """Enables the PWM signal/control mode on CPU2.

        Returns
        -------
        status : int
            Status of command. If command was executed successfully, returns 0.
            Otherwise, returns a negative integer.
            
        """
        funcname = Interface.cpu2_pwm_enable.__name__

        cmd = self.cmd.cpu2_pwm_enable
        
        self.ser.send(cmd)    
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tFailed to communicate with CPU1.'.format(funcname))
            return -1

        if data[0] == 1:
            print('{:}|\tCommunicated with CPU1 but CPU2 is unresponsive. PWM not enabled.'.format(funcname))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:], msb=True)

        if status != 0:
            print('{:}|\tCommand failed. Error: {:}.'.format(funcname, status))
            return -3

        print('{:}|\tPWM enabled.'.format(funcname))

        return 0


    def cpu2_pwm_disable(self):
        """Disables the PWM signal on CPU2.

        Returns
        -------
        status : int
            Status of command. If command was executed successfully, returns 0.
            Otherwise, returns a negative integer.
        
        """
        funcname = Interface.cpu2_pwm_disable.__name__
        cmd = self.cmd.cpu2_pwm_disable

        self.ser.send(cmd)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tFailed to communicate with CPU1.'.format(funcname))
            return -1

        if data[0] == 1:
            print('{:}|\tCommunicated with CPU1 but CPU2 is unresponsive. PWM not disabled.'.format(funcname))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:], msb=True)

        if status != 0:
            print('{:}|\tCommand failed. Error: {:}.'.format(funcname, status))
            return -3
        
        print('{:}|\tPWM disabled.'.format(funcname))

        return 0
    

    def cpu1_adc_buffer_set(self, adc, size):
        """Sets the ADC buffer.

        Parameters
        ----------
        adc : int
            ADC buffer to set. It must be an integer between 0 and `N-1`, where
            `N` is the maximum number of ADC buffers. The absolute maximum is
            set to 255, but the actual value which is limited by the hardware
            may be lower.

        size : int
            Buffer size, in number of samples. Absolute maximum is 65535, but
            the available memory on the platform may be less.

        Returns
        -------
        status : int
            Status of command. If command was executed successfully, returns 0.
            Otherwise, returns a negative integer.
        
        Raises
        ------
        TypeError
            If either `adc` or `size` is not of `int` type.

        ValueError
            If `adc` is not a value between 0 and 255.
        
        """
        funcname = Interface.cpu1_adc_buffer_set.__name__
        if type(adc) is not int or type(size) is not int:
            raise TypeError('`adc` and `size` must be of int type.')

        if adc > 255 or adc < 0:
            raise ValueError('`adc` must be a value between 0 and 255.')
        
        cmd = self.cmd.cpu1_adc_buffer_set

        adc = adc & 0xFF
        size8 = serialp.conversions.u16_to_u8(size & 0xFFFF, msb=True)

        data = [adc, size8[0], size8[1]]

        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tADC {:}: failed to communicate with CPU1.'.format(funcname, adc))
            return -1

        if data[0] != 0:
            print('{:}|\tADC {:}: command failed. Error: {:}.'.format(funcname, adc, data[0]))
            return -2

        adc_rcvd = serialp.conversions.u8_to_u16(data[1:3], msb=True)
        size_rcvd = serialp.conversions.u8_to_u16(data[3:], msb=True)

        if adc_rcvd != adc:
            print('{:}|\tADC {:}: error setting ADC. ADC sent: {:}. ADC received: {:}.'.format(funcname, adc, adc, adc_rcvd))
            return -3

        if size_rcvd != size:
            print('{:}|\tADC {:}: error setting size. Size sent: {:}. Size received: {:}.'.format(funcname, adc, size, size_rcvd))
            return -4
        
        print('{:}|\tADC {:}: buffer set. Size: {:}'.format(funcname, adc_rcvd, size_rcvd))

        return 0


    def cpu1_adc_buffer_read(self, adc):
        """Reads an ADC buffer.

        Parameters
        ----------
        adc : int
            ADC buffer to set. It must be an integer between 0 and `N-1`, where
            `N` is the maximum number of ADC buffers. The absolute maximum is
            set to 255, but the actual value which is limited by the hardware
            may be lower.

        Returns
        -------
        data or status : list or int
            ADC samples, as a list. The size of the list depends on how many
            samples were recorded, but will be at most `N`, where `N` is the
            buffer size. Also, this function can return a negative integer if
            an error occurred.

        Raises
        ------
        TypeError
            If `adc` is not of `int` type.

        ValueError
            If `adc` is not a value between 0 and 255.
            
        """
        funcname = Interface.cpu1_adc_buffer_read.__name__
        if type(adc) is not int:
            raise TypeError('`adc` must of of int type.')

        if adc > 255 or adc < 0:
            raise ValueError('`adc` must be a value between 0 and 255.')
        
        # Flushes input, in case we had any previous communication error
        while self.ser.serial.in_waiting != 0:
            self.ser.serial.flushInput()
            time.sleep(0.1)

        cmd = self.cmd.cpu1_adc_buffer_read
        data = [adc & 0xFF]

        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tADC {:}: failed to communicate with CPU1 or ADC buffer has been set to a size of 0.'.format(funcname, adc))
            return -1

        if len(data) == 1:
            print('{:}|\tADC {:}: command failed. Error: {:}.'.format(funcname, adc, data[0]))
            return -2           

        n = int( len(data) / 2 )
        print('{:}|\tADC {:}: data received. Samples: {:}'.format(funcname, adc, n))

        if n == 0:
            data = []
        else:
            data = [[data[2*i], data[2*i+1]] for i in range(n)]
            data = serialp.conversions.u8_to_u16(data, msb=False)
        
        return data


    def cpu2_buffer_set(self, buffer, size):
        """Sets CPU2's buffer.

        Parameters
        ----------
        buffer : int
            Buffer to set. It must be an integer between 0 and `N`, where `N`
            is the maximum number of CPU2 buffers. The absolute maximum is
            set to 255, but the actual value which is limited by the hardware
            may be lower.

        size : int
            Buffer size, in number of samples. Absolute maximum is 65535, but
            the available memory on the platform may be less.

        Returns
        -------
        status : int
            Status of command. If command was executed successfully, returns 0.
            Otherwise, returns a negative integer.
        
        Raises
        ------
        TypeError
            If either `buffer` or `size` is not of `int` type.

        ValueError
            If `buffer` is not a value between 0 and 255.

        ValueError
            If `size` is not a value between 0 and 65535.
        
        """
        funcname = Interface.cpu2_buffer_set.__name__
        if type(buffer) is not int or type(size) is not int:
            raise TypeError('`buffer` and `size` must be of int type.')

        if buffer > 255 or buffer < 0:
            raise ValueError('`buffer` must be a value between 0 and 255.')

        if size > 65535 or size < 0:
            raise ValueError('`size` must be a value between 0 and 65535.')
        
        cmd = self.cmd.cpu2_buffer_set

        buffer = buffer & 0xFF
        size8 = serialp.conversions.u16_to_u8(size & 0xFFFF, msb=True)

        data = [buffer, size8[0], size8[1]]

        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tBuffer {:}: failed to communicate with CPU1.'.format(funcname, buffer))
            return -1

        if data[0] != 0:
            print('{:}|\tBuffer {:}: command failed. Error: {:}.'.format(funcname, buffer, data[0]))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:5], msb=True)

        if status != 0:
            print('{:}|\tBuffer {:}: could not set buffer. Status: {:}.'.format(funcname, buffer, status))
            return -3

        buffer_rcvd = serialp.conversions.u8_to_u16(data[5:7], msb=True)
        size_rcvd = serialp.conversions.u8_to_u16(data[7:], msb=True)

        if buffer_rcvd != buffer:
            print('{:}|\tBuffer {:}: error setting buffer. Buffer sent: {:}. Buffer received: {:}.'.format(funcname, buffer, buffer, buffer_rcvd))
            return -3

        if size_rcvd != size:
            print('{:}|\tBuffer {:}: error setting size. Size sent: {:}. Size received: {:}.'.format(funcname, adc, size, size_rcvd))
            return -4
        
        print('{:}|\tBuffer {:}: buffer set. Size: {:}'.format(funcname, buffer_rcvd, size_rcvd))

        return 0

    
    def cpu2_buffer_read(self, buffer):
        """Reads CPU2's buffer.

        Parameters
        ----------
        buffer : int
            Buffer to read. It must be an integer between 0 and `N-1`, where
            `N` is the maximum number of buffers in CPU2. The absolute maximum is
            set to 255, but the actual value which is limited by the hardware
            may be lower.
            
        Returns
        -------
        data or status : list or int
            CPU2 buffer, as a list. The size of the list depends on how many
            samples were recorded, but will be at most `N`, where `N` is the
            buffer size. Also, this function can return a negative integer,
            in case the command fails.

        Raises
        ------
        TypeError
            If `buffer` is not of `int` type.

        ValueError
            If `buffer` is not a value between 0 and 255.
            
        """
        funcname = Interface.cpu2_buffer_read.__name__

        if type(buffer) is not int:
            raise TypeError('`buffer` must be of int type.')

        if buffer > 255 or buffer < 0:
            raise ValueError('`buffer` must be a value between 0 and 255.')
        
        # Flushes input, in case we had any previous communication error
        while self.ser.serial.in_waiting != 0:
            self.ser.serial.flushInput()
            time.sleep(0.1)

        cmd = self.cmd.cpu2_buffer_read

        data = [buffer & 0xFF]

        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tFailed to communicate with CPU1 or CPU2 buffer has been set to a size of 0.'.format(funcname))
            return -1

        if len(data) == 1:
            print('{:}|\tCommand failed. Error: {:}.'.format(funcname, data[0]))
            return -2

        n = int( len(data) / 2 )
        print('{:}|\tData received. Samples: {:}'.format(funcname, n))

        if n == 0:
            data = []
        else:
            data = [[data[2*i], data[2*i+1]] for i in range(n)]
            data = serialp.conversions.u8_to_u16(data, msb=False)
        
        return data


    def cpu2_control_mode_set(self, mode, params):
        """Sets the control mode.

        Parameters
        ----------
        mode : str
            Control mode.

        data : dict
            Additional data according to the control mode.

        Returns
        -------
        status : int
            Status of command. If command was executed successfully, returns 0.
            Otherwise, returns a negative integer.

        Raises
        ------
        TypeError
            If `mode` is not of `str` type.

        TypeError
            If `params` is not of `dict` type.
            
        TypeError
            If control mode is `ol`, raises TypeError if `u` is not of
            `float` type.

        ValueError
            If control mode is `ol`, raises ValueError if `u` is not a
            value between 0 and 1.

        TypeError
            If control mode is `pid`, raises TypeError if any of the gains is
            are not of `float` or `int` type.
            
        """
        funcname = Interface.cpu2_control_mode_set.__name__
        # Flushes input, in case we had any previous communication error
        while self.ser.serial.in_waiting != 0:
            self.ser.serial.flushInput()
            time.sleep(0.1)

        cmd = self.cmd.cpu2_control_mode_set

        if type(mode) is not str:
            raise TypeError('`mode` must be of `str` type.')

        if type(params) is not dict:
            raise TypeError('`params` must be of `dict` type.')

        if mode == 'ol':
            modei = 0
            u = params['u']
            
            if type(u) is not float:
                raise TypeError('In `ol` mode, `u` must be of float type.')

            if u > 1 or u < 0:
                raise ValueError('In `ol` mode, `u` must be between 0 and 1.')
            
            # Control mode
            data = [0]

            u_hex = list(struct.pack('f', u))[::-1]
            data.extend(u_hex)
            
        elif mode == 'pid':
            modei = 1
            a1 = params['a1']
            a2 = params['a2']
            b0 = params['b0']
            b1 = params['b1']
            b2 = params['b2']

            pid_gains = [a1, a2, b0, b1, b2]
            for g in pid_gains:
                if (type(g) is not float) and (type(g) is not int):
                    raise TypeError('In `pid` mode, gains a1, a2, b0, b1 and b2 must be of float type.')
            
            # Control mode 
            data = [1]

            g_hex = list(struct.pack('f', a1))[::-1]
            data.extend(g_hex)
            g_hex = list(struct.pack('f', a2))[::-1]
            data.extend(g_hex)
            g_hex = list(struct.pack('f', b0))[::-1]
            data.extend(g_hex)
            g_hex = list(struct.pack('f', b1))[::-1]
            data.extend(g_hex)
            g_hex = list(struct.pack('f', b2))[::-1]
            data.extend(g_hex)
            
        else:
            print('Mode not recognized')
            return -1

        self.ser.send(cmd, data)
        data = self.ser.read(cmd)
        
        if data == []:
            print('{:}|\tControl mode {:}: failed to communicate with CPU1.'.format(funcname, mode))
            return -1

        if data[0] != 0:
            print('{:}|\tControl mode {:}: command failed. Error: {:}.'.format(funcname, mode, data[0]))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:5], msb=True)   

        if status != 0:
            print('{:}|\tControl mode {:}: could not set mode. Status: {:}.'.format(funcname, mode, status))
            return -3

        mode_rcvd = serialp.conversions.u8_to_u16(data[5:], msb=True)

        if mode_rcvd != modei:
            print('{:}|\tControl mode {:}: could not set mode. Mode sent: {:}. Mode received: {:}.'.format(funcname, modei, modei, mode_rcvd))
            return -4
        
        print('{:}|\tControl mode {:}: mode set.'.format(funcname, mode_rcvd))

        return status


    def cpu2_control_mode_read(self):
        """Gets the control mode.

        Parameters
        ----------
        mode : str
            Control mode.

        data : dict
            Additional data according to the control mode.

        Returns
        -------
        mode : int
            Control mode, if received correctly. Otherwise, returns a
            negative integer.
            
        """
        funcname = Interface.cpu2_control_mode_read.__name__

        cmd = self.cmd.cpu2_control_mode_read
        self.ser.send(cmd)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tFailed to communicate with CPU1.'.format(funcname))
            return -1
        
        if data[0] == 1:
            print('{:}|\tCommunicated with CPU1 but CPU2 is unresponsive. Control mode not read.'.format(funcname))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:5], msb=True)   

        if status != 0:
            print('{:}|\tCould not read control mode. Status: {:}.'.format(funcname, status))
            return -3

        mode = serialp.conversions.u8_to_u16(data[5:], msb=True)
        print('{:}|\tCould mode: {:}.'.format(funcname, mode))
        
        return mode
        

    def cpu2_ref_set(self, ref):
        """Sets the reference.

        Parameters
        ----------
        ref : int
            Reference.

        Returns
        -------
        status : int
            Status of command. If command was executed successfully, returns 0.
            Otherwise, returns a negative integer.

        Raises
        ------
        TypeError
            If `ref` is not of `int` type.

        ValueError
            If `ref` is not a value between 0 and 4095.
            
        """
        funcname = Interface.cpu2_ref_set.__name__
        # Flushes input, in case we had any previous communication error
        while self.ser.serial.in_waiting != 0:
            self.ser.serial.flushInput()
            time.sleep(0.1)

        cmd = self.cmd.cpu2_ref_set

        if type(ref) is not int:
            raise TypeError('`ref` must be of `int` type.')

        if ref > 4095 or ref < 0:
            raise TypeError('`ref` must be a value between 0 and 4095.')

        ref8 = serialp.conversions.u16_to_u8(ref, msb=True)

        self.ser.send(cmd, ref8)
        data = self.ser.read(cmd)
        
        if data == []:
            print('{:}|\tSet ref {:}: failed to communicate with CPU1.'.format(funcname, ref))
            return -1

        if data[0] != 0:
            print('{:}|\tSet ref {:}: command failed. Error: {:}.'.format(funcname, ref, data[0]))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:5], msb=True)

        if status != 0:
            print('{:}|\tSet ref {:}: could not set reference. Status: {:}.'.format(funcname, ref, status))
            return -3

        ref_rcvd = serialp.conversions.u8_to_u16(data[5:], msb=True)

        if ref_rcvd != ref:
            print('{:}|\tSet ref {:}: could not set reference. Ref sent: {:}. Ref received: {:}.'.format(funcname, ref, ref, ref_rcvd))
            return -4

        print('{:}|\tSet ref {:}: reference set.'.format(funcname, ref_rcvd))

        return status
    

    def cpu2_ref_read(self):
        """Gets the reference.

        Returns
        -------
        status : int
            Reference, if received correctly. Otherwise, returns a negative
            integer.
            
        """
        funcname = Interface.cpu2_ref_read.__name__

        cmd = self.cmd.cpu2_ref_read
        self.ser.send(cmd)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tFailed to communicate with CPU1.'.format(funcname))
            return -1
        
        if data[0] == 1:
            print('{:}|\tCommunicated with CPU1 but CPU2 is unresponsive.'.format(funcname))
            return -1

        status = serialp.conversions.u8_to_u16(data[1:5], msb=True)

        if status != 0:
            print('{:}|\tError reading reference. Error: {:}.'.format(funcname, status))

        ref = serialp.conversions.u8_to_u16(data[5:], msb=True)
        print('{:}|\tReference: {:}'.format(funcname, ref))

        return ref


    def cpu2_trip_set(self, adc, trip):
        """Sets the higher trip value for an ADC.

        Parameters
        ----------
        adc : int
            ADC to set tripping value. It must be an integer between 0 and
            `N-1`, where `N` is the maximum number of ADCs.

        trip : int
            Tripping value.

        Returns
        -------
        status : int
            Status of command. If command was executed successfully, returns 0.
            Otherwise, returns a negative integer.

        Raises
        ------
        TypeError
            If `adc` is not of `int` type.

        TypeError
            If `trip` is not of `int` type.

        ValueError
            If `trip` is not a value between 0 and 4095.
        """
        funcname = Interface.cpu2_trip_set.__name__

        if type(adc) is not int:
            raise TypeError('`adc` must be of `int` type.')

        if type(trip) is not int:
            raise TypeError('`trip` must be of `int` type.')

        if trip > 4095 or trip < 0:
            raise ValueError('`trip` must be a value between 0 and 4095.')


        adc = adc & 0xFF
        trip8 = serialp.conversions.u16_to_u8(trip & 0xFFFF, msb=True)

        data = [adc, trip8[0], trip8[1]]
        
        cmd = self.cmd.cpu2_trip_set
        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tADC {:} set trip {:}: failed to communicate with CPU1.'.format(funcname, adc, trip))
            return -1

        if data[0] != 0:
            print('{:}|\tADC {:} set trip {:}: command failed. Error: {:}.'.format(funcname, adc, trip, ref, data[0]))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:5], msb=True)

        if status != 0:
            print('{:}|\tADC {:} set trip {:}: could not set trip. Status: {:}.'.format(funcname, adc, trip,  status))
            return -3

        trip_rcvd = serialp.conversions.u8_to_u16(data[5:7], msb=True)
        adc_rcvd = serialp.conversions.u8_to_u16(data[7:], msb=True)

        if adc_rcvd != adc:
            print('{:}|\tADC {:} set trip {:}: could not set trip. ADC sent: {:}. ADC received: {:}.'.format(funcname, adc, trip, adc, adc_rcvd))
            return -4

        if trip_rcvd != trip:
            print('{:}|\tADC {:} set trip {:}: could not set trip. Trip sent: {:}. Trip received: {:}.'.format(funcname, adc, trip, trip, trip_rcvd))
            return -5
        
        print('{:}|\tADC {:} set trip {:}: trip value set.'.format(funcname, adc_rcvd, trip_rcvd))

        return status



    def cpu2_trip_enable(self, adc):
        """Enables tripping for an ADC.

        Parameters
        ----------
        adc : int
            ADC to set enable tripping. It must be an integer between 0 and
            `N-1`, where `N` is the maximum number of ADCs.

        Returns
        -------
        status : int
            Status of command. If command was executed successfully, returns 0.
            Otherwise, returns a negative integer.

        Raises
        ------
        TypeError
            If `adc` is not of `int` type.

        """
        funcname = Interface.cpu2_trip_enable.__name__

        if type(adc) is not int:
            raise TypeError('`adc` must be of `int` type.')

        adc = adc & 0xFF

        data = [adc]
        
        cmd = self.cmd.cpu2_trip_enable
        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tADC {:} trip enable: failed to communicate with CPU1.'.format(funcname, adc))
            return -1

        if data[0] != 0:
            print('{:}|\tADC {:} trip enable: command failed. Error: {:}.'.format(funcname, adc, data[0]))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:5], msb=True)

        if status != 0:
            print('{:}|\tADC {:} trip enable: could not enable trip. Status: {:}.'.format(funcname, adc, status))
            return -3

        adc_rcvd = serialp.conversions.u8_to_u16(data[5:], msb=True)

        if adc_rcvd != adc:
            print('{:}|\tADC {:} trip enable: could not enable trip. ADC sent: {:}. ADC received: {:}.'.format(funcname, adc, adc, adc_rcvd))
            return -4
        
        print('{:}|\tADC {:} trip enable: trip enabled.'.format(funcname, adc_rcvd))

        return status


    def cpu2_trip_disable(self, adc):
        """Disables tripping for an ADC.

        Parameters
        ----------
        adc : int
            ADC to disable tripping. It must be an integer between 0 and
            `N-1`, where `N` is the maximum number of ADCs.

        Returns
        -------
        status : int
            Status of command. If command was executed successfully, returns 0.
            Otherwise, returns a negative integer.

        Raises
        ------
        TypeError
            If `adc` is not of `int` type.

        """
        funcname = Interface.cpu2_trip_disable.__name__

        if type(adc) is not int:
            raise TypeError('`adc` must be of `int` type.')

        adc = adc & 0xFF

        data = [adc]
        
        cmd = self.cmd.cpu2_trip_disable
        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tADC {:} trip disable: failed to communicate with CPU1.'.format(funcname, adc))
            return -1

        if data[0] != 0:
            print('{:}|\tADC {:} trip disable: command failed. Error: {:}.'.format(funcname, adc, data[0]))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:5], msb=True)

        if status != 0:
            print('{:}|\tADC {:} trip disable: could not disable trip. Status: {:}.'.format(funcname, adc, status))
            return -3

        adc_rcvd = serialp.conversions.u8_to_u16(data[5:], msb=True)

        if adc_rcvd != adc:
            print('{:}|\tADC {:} trip disable: could not disable trip. ADC sent: {:}. ADC received: {:}.'.format(funcname, adc, adc, adc_rcvd))
            return -4
        
        print('{:}|\tADC {:} trip disable: trip disabled.'.format(funcname, adc_rcvd))

        return status


    def cpu2_trip_read(self, adc):
        """Reads the tripping value for an ADC.

        Parameters
        ----------
        adc : int
            ADC to read tripping value. It must be an integer between 0 and
            `N-1`, where `N` is the maximum number of ADCs.

        Returns
        -------
        value, status : int
            Tripping value, as a positive integer. A negative integer is
            returned if an error occurred.

        Raises
        ------
        TypeError
            If `adc` is not of `int` type.

        """
        funcname = Interface.cpu2_trip_disable.__name__

        if type(adc) is not int:
            raise TypeError('`adc` must be of `int` type.')

        adc = adc & 0xFF

        data = [adc]
        
        cmd = self.cmd.cpu2_trip_read
        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tADC {:} trip read: failed to communicate with CPU1.'.format(funcname, adc))
            return -1

        if data[0] != 0:
            print('{:}|\tADC {:} trip read: command failed. Error: {:}.'.format(funcname, adc, data[0]))
            return -2

        status = serialp.conversions.u8_to_u16(data[1:5], msb=True)

        if status != 0:
            print('{:}|\tADC {:} trip read: could not read trip. Status: {:}.'.format(funcname, adc, status))
            return -3

        ref_rcvd = serialp.conversions.u8_to_u16(data[5:7], msb=True)
        adc_rcvd = serialp.conversions.u8_to_u16(data[7:], msb=True)

        if adc_rcvd != adc:
            print('{:}|\tADC {:} trip read: could not read trip. ADC sent: {:}. ADC received: {:}.'.format(funcname, adc, adc, adc_rcvd))
            return -4
        
        print('{:}|\tADC {:} trip read: trip read. Value: {:}'.format(funcname, adc_rcvd, ref_rcvd))

        return ref_rcvd
