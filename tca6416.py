from machine import I2C
from machine import Pin
import time

class GPIO_Pin:
    P0_0 = (1 << 0) 
    P0_1 = (1 << 1)
    P0_2 = (1 << 2)
    P0_3 = (1 << 3)
    P0_4 = (1 << 4)
    P0_5 = (1 << 5)
    P0_6 = (1 << 6)
    P0_7 = (1 << 7)

    P1_0 = (1 << 8)
    P1_1 = (1 << 9)
    P1_2 = (1 << 10) 
    P1_3 = (1 << 11)
    P1_4 = (1 << 12)
    P1_5 = (1 << 13)
    P1_6 = (1 << 14) 
    P1_7 = (1 << 15)

class Mode:
    OUTPUT = 0
    INPUT = 1

class Value:
    HIGH = 1  
    LOW = 0

class TCA6416():
    def __init__(self, address=0x20):
        # I2C bus
        self.i2c = I2C(0, scl = Pin(32), sda = Pin(33), freq = 100000)

        # i2c address
        self.address = address

        # Control registers address 
        self.INPUT_PORT_0_ADDR = 0x00 
        self.INPUT_PORT_1_ADDR = 0x01
        self.OUTPUT_PORT_0_ADDR = 0x02
        self.OUTPUT_PORT_1_ADDR = 0x03
        self.POLARITY_INV_PORT_0_ADDR = 0x04
        self.POLARITY_INV_PORT_1_ADDR = 0x05
        self.CONF_PORT_0_ADDR = 0x06
        self.CONF_PORT_1_ADDR = 0x07

        # Control register values(default)
        self.INPUT_PORT_0 = 0x00 
        self.INPUT_PORT_1 = 0x00
        self.OUTPUT_PORT_0 = 0xFF
        self.OUTPUT_PORT_1 = 0xFF
        self.POLARITY_INV_PORT_0 = 0x00
        self.POLARITY_INV_PORT_1 = 0x00
        self.CONF_PORT_0 = 0xFF
        self.CONF_PORT_1 = 0xFF

        # Read registers from device
        # Input Port 0 
        self.INPUT_PORT_0 = self.read_register(self.INPUT_PORT_0_ADDR)
        print("INPUT_PORT_0 = 0x{:02x}".format(self.INPUT_PORT_0))

        # Input Port 1
        self.INPUT_PORT_1 = self.read_register(self.INPUT_PORT_1_ADDR)
        print("INPUT_PORT_1 = 0x{:02x}".format(self.INPUT_PORT_1))

        # Output Port 0
        self.OUTPUT_PORT_0 = self.read_register(self.OUTPUT_PORT_0_ADDR)
        print("OUTPUT_PORT_0 = 0x{:02x}".format(self.OUTPUT_PORT_0)) 

        # Output Port 1
        self.OUTPUT_PORT_1 = self.read_register(self.OUTPUT_PORT_1_ADDR)
        print("OUTPUT_PORT_1 = 0x{:02x}".format(self.OUTPUT_PORT_1))

        # Polarity Inversion Port 0 
        self.POLARITY_INV_PORT_0 = self.read_register(self.POLARITY_INV_PORT_0_ADDR)
        print("POLARITY_INV_PORT_0 = 0x{:02x}".format(self.POLARITY_INV_PORT_0))

        # Polarity Inversion Port 1
        self.POLARITY_INV_PORT_1 = self.read_register(self.POLARITY_INV_PORT_1_ADDR)
        print("POLARITY_INV_PORT_1 = 0x{:02x}".format(self.POLARITY_INV_PORT_1))

        # Configuration Port 0
        self.CONF_PORT_0 = self.read_register(self.CONF_PORT_0_ADDR)
        print("CONF_PORT_0 = 0x{:02x}".format(self.CONF_PORT_0))

        # Configuration Port 1
        self.CONF_PORT_1 = self.read_register(self.CONF_PORT_1_ADDR)
        print("CONF_PORT_1 = 0x{:02x}".format(self.CONF_PORT_1))


    def pin_mode(self, pin, mode):
        print("--- Pin Mode ---")
        
        p = int(pin)
        m = int(mode)

        # PORT_1 (P1_0, ... P1_7)        
        if p > 255:
            print(">PORT_1")
            p = p >> 8
            # mode
            if m:
                # input mode
                self.CONF_PORT_1 = self.CONF_PORT_1 | p
                print(">>>CONF_PORT_1: 0b{:08b}".format(self.CONF_PORT_1))
            else:
                # output mode
                self.CONF_PORT_1 = self.CONF_PORT_1 & ~p        
                print(">>>CONF_PORT_1: 0b{:08b}".format(self.CONF_PORT_1))
            self.write_register(self.CONF_PORT_1_ADDR, self.CONF_PORT_1)
        # PORT_0 (P0_0, ... P0_7)
        else:
            # mode
            print(">PORT_0")
            if m:
                # input mode
                self.CONF_PORT_0 = self.CONF_PORT_0 | p
                print(">>>CONF_PORT_0: 0b{:08b}".format(self.CONF_PORT_0))

            else:
                # output mode
                self.CONF_PORT_0 = self.CONF_PORT_0 & ~p
                print(">>>CONF_PORT_0: 0b{:08b}".format(self.CONF_PORT_0))
            self.write_register(self.CONF_PORT_0_ADDR, self.CONF_PORT_0)
                

    def digital_write(self, pin, value):
        #print("digital_write")
        p = int(pin)
        v = int(value)
        # PORT_1 (P1_0, ... P1_7)        
        if p > 255:
            p = p >> 8
            if v:
                # high value
                self.OUTPUT_PORT_1 = self.OUTPUT_PORT_1 | p
            else:
                # low value
                self.OUTPUT_PORT_1 = self.OUTPUT_PORT_1 & ~p
            self.write_register(self.OUTPUT_PORT_1_ADDR, self.OUTPUT_PORT_1)
        
        # PORT_0 (P0_0, ... P0_7)
        else:
            # value
            if v:
                # high value
                self.OUTPUT_PORT_0 = self.OUTPUT_PORT_0 | p    
            else:
                # low value
                self.OUTPUT_PORT_0 = self.OUTPUT_PORT_0 & ~p        
            self.write_register(self.OUTPUT_PORT_0_ADDR, self.OUTPUT_PORT_0)

    def digital_read(self, pin):
        p = int(pin)
        
        # PORT_1 (P1_0, ... P1_7)        
        if p > 255:
            p = p >> 8
            # Read I2C PORT_1
            self.INPUT_PORT_1 = self.read_register(self.INPUT_PORT_1_ADDR)
            # print("INPUT_PORT_1 = 0x{:02x}".format(self.INPUT_PORT_1))
            r = 1 if (self.INPUT_PORT_1 & p) > 0 else 0
            return r
        # PORT_0 (P0_0, ... P0_7)
        else:
            # Read I2C PORT_0
            self.INPUT_PORT_0 = self.read_register(self.INPUT_PORT_0_ADDR)
            # print("INPUT_PORT_0 = 0x{:02x}".format(self.INPUT_PORT_0))
            r = 1 if (self.INPUT_PORT_0 & p) > 0 else 0
            return r


    def write_register(self, register, value, wait = True):
        self.i2c.writeto_mem(self.address, register, bytearray([value]))
        if wait:
            time.sleep(0.005) #sleep for 5ms

    def read_register(self, register):
        return self.i2c.readfrom_mem(self.address, register, 1)[0]