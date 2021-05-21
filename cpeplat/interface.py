import serialp

class Interface:

    def __init__(self, com, baud, to):
        self.ser = serialp.serial.Serial(com, baud, to)


    def blink(self, t):
        t = serialp.conversions.u16_to_u8(t, msb=True)

        self.ser.send(0x01, t)
        
        
    def relay_1(self, state):

        if state is True:
            r = 1
        else:
            r = 0
            
        self.ser.send(0x02, [r])

    
    def relay_2(self, state):

        if state is True:
            r = 1
        else:
            r = 0
            
        self.ser.send(0x03, [r])


    def adcConv(self):
            
        self.ser.send(0x04, [])


    def adcRead(self):
            
        self.ser.send(0x05, [])
        d = self.ser.read(0x05)

        n = int( len(d) / 2 )
        d = [[d[2*i], d[2*i+1]] for i in range(n)]

        d = serialp.conversions.u8_to_u16(d, msb=False)

        return d
        
