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
        

class Interface:
    """A class to provide an interface to the C2000-based platform.

    Parameters
    ----------
    com : str
        COM port.

    baud : int
        Baudrate used for communication. By default, it is 9600 bps.

    timeout : int, float
        Communication time-out, in seconds. By default, it is 0.2 s.
        
    """
    def __init__(self, com, baud, to):
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
        int
            Status of CPU2, if received correctly. Otherwise, returns -1.
            
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
            return -1

        status = serialp.conversions.u8_to_u16(data[1:], msb=True)
        print('{:}|\tCPU2 status: {:}'.format(funcname, status))

        return status


    def cpu2_status_clear(self):
        """Clears the status of CPU2.

        Returns
        -------
        int
            Status of CPU2, which should be zero if the command worked.
            Returns -1 if failed to communicate with CPU1 or a positive
            integer if CPU1 returned an error.
        
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
            return 1

        status = serialp.conversions.u8_to_u16(data[1:], msb=True)        

        if status != 0:
            print('{:}|\tCommand failed. CPU2 status {:}.'.format(funcname, status))
            return 1

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
        int
            Status of command. If command was executed successfully, returns 0.
            If there was an issue, will return -1 if failed to communicate with
            CPU1 and a positive integer if CPU1 returned an error.
        
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
            return data[0]

        print('{:}|\tGPIO {:}: GPIO set. State: {:}.'.format(funcname, gpio, state))

        return 0


    def cpu2_pwm_enable(self, mode, params):
        """Enables the PWM signal on CPU2, with the specified duty cycle.

        Parameters
        ----------
        mode : str
            Control mode.

        data : dict
            Additional data according to the control mode.

        Returns
        -------
        int
            Status of command. If command was executed successfully, returns 0.
            If there was an issue, will return -1 if failed to communicate with
            CPU1 and a positive integer if CPU1 returned an error.
            
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
            If control mode is `ol`, raises TypeError if `ref` is not of
            `int` type.

        ValueError
            If control mode is `ol`, raises ValueError if `ref` is not a
            value between 0 and 4095.
            
        TypeError
            If control mode is `pid`, raises TypeError if `data` is not of
            `int` type.

        ValueError
            If control mode is `ol`, raises ValueError if `data` is not a
            value between 0 and 4095.
            
        """
        funcname = Interface.cpu2_pwm_enable.__name__
        if type(mode) is not str:
            raise TypeError('`mode` must be of `str` type.')

        if type(params) is not dict:
            raise TypeError('`params` must be of `dict` type.')

        if mode == 'ol':
            u = params['u']
            ref = params['ref']
            
            if type(u) is not float:
                raise TypeError('In `ol` mode, `u` must be of float type.')

            if u > 1 or u < 0:
                raise ValueError('In `ol` mode, `u` must be between 0 and 1.')

            if type(ref) is not int:
                raise ValueError('In `ol` mode, `ref` must be of int type.')

            if ref > 4095 or ref < 0:
                raise ValueError('In `ol` mode, `ref` must be between 0 and 4095.')
            
            cmd = self.cmd.cpu2_pwm_enable

            # Control mode
            data = [0]

            ref_hex = serialp.conversions.u32_to_u8(ref, msb=True)
            data.extend(ref_hex)

            u_hex = list(struct.pack('f', u))[::-1]
            data.extend(u_hex)
            
            self.ser.send(cmd, data)


        elif mode == 'pid':
            ref = params['ref']
            a1 = params['a1']
            a2 = params['a2']
            b0 = params['b0']
            b1 = params['b1']
            b2 = params['b2']

            if type(ref) is not int:
                raise ValueError('In `pid` mode, `ref` must be of int type.')

            if ref > 4095 or ref < 0:
                raise ValueError('In `pid` mode, `ref` must be between 0 and 4095.')

            pid_gains = [a1, a2, b0, b1, b2]
            for g in pid_gains:
                if (type(g) is not float) and (type(g) is not int):
                    raise TypeError('In `pid` mode, gains a1, a2, b0, b1 and b2 must be of float type.')
            
            cmd = self.cmd.cpu2_pwm_enable

            # Control mode 
            data = [1]

            ref_hex = serialp.conversions.u32_to_u8(ref, msb=True)
            data.extend(ref_hex)

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
            
            self.ser.send(cmd, data)

        else:
            print('Mode not recognized')
            return -1
            
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tFailed to communicate with CPU1.'.format(funcname))
            return -1

        if data[0] == 1:
            print('{:}|\tCommunicated with CPU1 but CPU2 is unresponsive. PWM not set.'.format(funcname))
            return data[0]

        status = serialp.conversions.u8_to_u16(data[1:], msb=True)   

        if status != 0:
            print('{:}|\tCommand failed. Error: {:}.'.format(funcname, status))
            return status

        print('{:}|\tControl mode set. Mode: {:}'.format(funcname, mode))

        return 0


    def cpu2_pwm_disable(self):
        """Disables the PWM signal on CPU2.

        Returns
        -------
        int
            Status of command. If command was executed successfully, returns 0.
            If there was an issue, will return -1 if failed to communicate with
            CPU1 and a positive integer if CPU1 returned an error.
        
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
            return data[0]            

        print('{:}|\tPWM disabled.'.format(funcname))

        return 0
    

    def cpu1_adc_buffer_set(self, adc, size):
        """Sets the ADC buffer.

        Parameters
        ----------
        adc : int
            ADC buffer to set. It must be an integer between 0 and `N`, where
            `N` is the maximum number of ADC buffers.

        size : int
            Buffer size, in number of samples.

        Returns
        -------
        int
            Returns 0 if the command was processed successfully. Returns -1 if
            could not communicate with CPU1 and returns a positive integer if
            there was any other error (ADC number or buffer size).
        
        Raises
        ------
        TypeError
            If either `adc` or `size` is not of `int` type.
        
        """
        funcname = Interface.cpu1_adc_buffer_set.__name__
        if type(adc) is not int or type(size) is not int:
            raise TypeError('`adc` and `size` must be of int type.')
        
        cmd = self.cmd.cpu1_adc_buffer_set

        adc = adc & 0xFF
        size8 = serialp.conversions.u16_to_u8(size, msb=True)

        data = [adc, size8[0], size8[1]]

        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tADC {:}: failed to communicate with CPU1.'.format(funcname, adc))
            return -1

        if data[0] != 0:
            print('{:}|\tADC {:}: command failed. Error: {:}.'.format(funcname, adc, data[0]))
            return data[0]

        print('{:}|\tADC {:}: buffer set. Size: {:}'.format(funcname, adc, size))

        return data[0]


    def cpu1_adc_buffer_read(self, adc):
        """Reads an ADC buffer.

        Parameters
        ----------
        adc : int
            ADC buffer to set. It must be an integer between 0 and `N`, where
            `N` is the maximum number of ADC buffers.

        Returns
        -------
        list, int
            ADC samples, as a list. The size of the list depends on how many
            samples were recorded, but will be at most `N`, where `N` is the
            buffer size. Also, this function can return an integer. It returns
            -1 if failed to communicate with CPU1 or a positive integer if
            the command failed.

        Raises
        ------
        TypeError
            If `adc` is not of `int` type.
            
        """
        funcname = Interface.cpu1_adc_buffer_read.__name__
        if type(adc) is not int:
            raise TypeError('`adc` must of of int type.')
        
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
            return data[0]            

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
            is the maximum number of CPU2 buffers.

        size : int
            Buffer size, in number of samples.

        Returns
        -------
        int
            Returns 0 if the command was processed successfully. Returns -1 if
            could not communicate with CPU1 and returns a positive integer if
            there was any other error (buffer number or buffer size).
        
        Raises
        ------
        TypeError
            If either `buffer` or `size` is not of `int` type.
        
        """
        funcname = Interface.cpu2_buffer_set.__name__
        if type(buffer) is not int or type(size) is not int:
            raise TypeError('`buffer` and `size` must be of int type.')
        
        cmd = self.cmd.cpu2_buffer_set

        buffer = buffer & 0xFF
        size8 = serialp.conversions.u16_to_u8(size, msb=True)

        data = [buffer, size8[0], size8[1]]

        self.ser.send(cmd, data)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tBuffer {:}: failed to communicate with CPU1.'.format(funcname, buffer))
            return -1

        if data[0] != 0:
            print('{:}|\tBuffer {:}: command failed. Error: {:}.'.format(funcname, buffer, data[0]))
            return data[0]

        status = serialp.conversions.u8_to_u16(data[1:], msb=True)   

        if status != 0:
            print('{:}|\tBuffer {:}: could not set buffer. Status: {:}.'.format(funcname, buffer, status))
            return status
        
        print('{:}|\tBuffer {:}: buffer set. Size: {:}'.format(funcname, buffer, size))

        return data[0]

    
    def cpu2_buffer_read(self):
        """Reads CPU2's buffer.

        Returns
        -------
        list, int
            CPU2 buffer, as a list. The size of the list depends on how many
            samples were recorded, but will be at most `N`, where `N` is the
            buffer size. Also, this function can return an integer. It returns
            -1 if failed to communicate with CPU1 or a positive integer if
            the command failed.
            
        """
        funcname = Interface.cpu2_buffer_read.__name__
        # Flushes input, in case we had any previous communication error
        while self.ser.serial.in_waiting != 0:
            self.ser.serial.flushInput()
            time.sleep(0.1)

        cmd = self.cmd.cpu2_buffer_read

        self.ser.send(cmd)
        data = self.ser.read(cmd)

        if data == []:
            print('{:}|\tFailed to communicate with CPU1 or CPU2 buffer has been set to a size of 0.'.format(funcname))
            return -1

        if len(data) == 1:
            print('{:}|\tCommand failed. Error: {:}.'.format(funcname, data[0]))
            return data[0]

        n = int( len(data) / 2 )
        print('{:}|\tData received. Samples: {:}'.format(funcname, n))

        if n == 0:
            data = []
        else:
            data = [[data[2*i], data[2*i+1]] for i in range(n)]
            data = serialp.conversions.u8_to_u16(data, msb=False)
        
        return data
