import serialp
import time

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
            raise TypeError('`t` must be of int type')
        
        t = serialp.conversions.u16_to_u8(t, msb=True)
        cmd = self.cmd.cpu1_blink

        self.ser.send(cmd, t)
        

    def cpu2_status(self):
        """Gets the status of CPU2.

        Returns
        -------
        int
            Status of CPU2.
            
        """
        cmd = self.cmd.cpu2_status

        self.ser.send(cmd)
        status = self.ser.read(cmd)
        status = serialp.conversions.u8_to_u16(status, msb=True)

        return status


    def cpu2_status_clear(self):
        """Clears the status of CPU2.            
        """
        cmd = self.cmd.cpu2_status_clear

        self.ser.send(cmd)
    
        
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
            raise TypeError('`t` must be of int type')
        
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

        Raises
        ------
        TypeError
            If either `gpio` or `state` is not of `int` type.
        
        """
        if type(gpio) is not int or type(state) is not int:
            raise TypeError('`gpio` and `state` must be of int type')

        data = [gpio, state]
        cmd = self.cmd.cpu2_gpio

        self.ser.send(cmd, data)


    def cpu2_pwm_enable(self, dc):
        """Enables the PWM signal on CPU2, with the specified duty cycle.

        Parameters
        ----------
        dc : float
            The duty cycle, as a value between 0 and 1.

        Raises
        ------
        TypeError
            If `dc` is not of `float` type.

        ValueError
            If `dc` is not a value between 0 and 1.
            
        """
        if type(dc) is int:
            if dc == 0: dc = 0.0
        
        if type(dc) is not float:
            raise TypeError('`dc` must be of float type')

        if dc > 1 or dc < 0:
            raise ValueError('`dc` must be between 0 and 1')
        
        cmd = self.cmd.cpu2_pwm_enable
        dc = int(dc * 499)
        print('Duty cycle: {:}'.format(dc))

        data = serialp.conversions.u16_to_u8(dc, msb=True)
        print('TX data: {:}'.format(data))

        self.ser.send(cmd, data)


    def cpu2_pwm_disable(self):
        """Disables the PWM signal on CPU2.
        """
        cmd = self.cmd.cpu2_pwm_disable

        self.ser.send(cmd)


    def cpu1_adc_set_buffer(self, adc, size):
        """Sets the ADC buffer.

        Parameters
        ----------
        adc : int
            ADC buffer to set. It must be an integer between 0 and `N`, where
            `N` is the maximum number of ADC buffers.

        size : int
            Buffer size, in number of samples.

        Raises
        ------
        TypeError
            If either `adc` or `size` is not of `int` type.
        
        """
        if type(adc) is not int or type(size) is not int:
            raise TypeError('`adc` and `size` must be of int type')
        
        cmd = self.cmd.cpu1_adc_buffer_set

        adc = adc & 0xFF
        size = serialp.conversions.u16_to_u8(size, msb=True)

        data = [adc, size[0], size[1]]

        self.ser.send(cmd, data)
    

    def cpu1_adc_read_buffer(self, adc):
        """Reads an ADC buffer.

        Parameters
        ----------
        adc : int
            ADC buffer to set. It must be an integer between 0 and `N`, where
            `N` is the maximum number of ADC buffers.

        Returns
        -------
        list
            ADC samples. The size of the list depends on how many samples were
            recorded, but will be at most `N`, where `N` is the buffer size.

        Raises
        ------
        TypeError
            If `adc` is not of `int` type.
            
        """
        if type(adc) is not int:
            raise TypeError('`adc` must of of int type')
        
        # Flushes input, in case we had any previous communication error
        while self.ser.serial.in_waiting != 0:
            self.ser.serial.flushInput()
            time.sleep(0.1)

        cmd = self.cmd.cpu1_adc_buffer_read
        data = [adc & 0xFF]

        self.ser.send(cmd, data)
        d = self.ser.read(cmd)

        if d:
            print('ADC read: data received')
            n = int( len(d) / 2 )
            d = [[d[2*i], d[2*i+1]] for i in range(n)]
            d = serialp.conversions.u8_to_u16(d, msb=False)
        else:
            print('ADC read: failed')
            d = []
        
        return d
