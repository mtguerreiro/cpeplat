import serialp
import time

"""
Ideas
-----
For the interface module, we could only have generic commands. For instance,
one command could be to initialize a GPIO, and set/reset its level. Then,
we could create another module, which would be specific for a given
application. One would be the buck converter, where some GPIOs are dedicated
to specific stuff (relays, for instance).

In this way, we keep this module generic, and the other module is application-
specific.

"""

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


    def blink(self, t=1000):
        """Changes the LED's blinking period.

        Parameters
        ----------
        t : int
            Period, in milliseconds. By default, it is 1000.

        """
        t = serialp.conversions.u16_to_u8(t, msb=True)

        self.ser.send(0x01, t)
        
        
    def relay_1(self, state):
        """Sets the state of relay 1.

        Parameters
        ----------
        state : bool
            State. If `True`, the relay is turned-on. If `False`, the relay is
            turned off.

        """
        if state is True:
            r = 1
        else:
            r = 0
            
        self.ser.send(0x02, [r])

    
    def relay_2(self, state):
        """Sets the state of relay 2.

        Parameters
        ----------
        state : bool
            State. If `True`, the relay is turned-on. If `False`, the relay is
            turned off.

        """
        if state is True:
            r = 1
        else:
            r = 0
            
        self.ser.send(0x03, [r])


    def adcConv(self):
        """Converts and receives samples from the ADC. This is just a test.

        Returns
        -------
        list
            List with the ADC measurements.
        """
        self.ser.send(0x04)

        time.sleep(0.2)

        self.ser.send(0x05)
        d = self.ser.read(0x05)

        n = int( len(d) / 2 )
        d = [[d[2*i], d[2*i+1]] for i in range(n)]

        d = serialp.conversions.u8_to_u16(d, msb=False)

        return d


    def adcRead(self):
        """

        """
        self.ser.send(0x05, [])
        d = self.ser.read(0x05)

        n = int( len(d) / 2 )
        d = [[d[2*i], d[2*i+1]] for i in range(n)]

        d = serialp.conversions.u8_to_u16(d, msb=False)

        return d
        
