from machine import I2C
from machine import Pin
import time

class Value:
    HIGH = 1  
    LOW = 0

class BQ25710():
    def __init__(self, address=0x09):
        # I2C bus
        self.i2c = I2C(0, scl = Pin(32), sda = Pin(33), freq = 100000)

        # i2c address
        self.address = address

        # Control registers address 
        self.MAX_CHARGE_VOLTAGE_REG = 0x15 
        self.CHARGE_CURRENT_REG = 0x14

    def write_register(self, register, value, wait = True):
        towrite = bytearray([value & 0xff, (value >> 8) & 0xff])
        self.i2c.writeto_mem(self.address, register, towrite)
        if wait:
            time.sleep(0.005) #sleep for 5ms

    def read_register(self, register):
        reg = self.i2c.readfrom_mem(self.address, register, 2)
        return reg[0] + (reg[1] * 256)

    def setMaxChargeVoltage(self, voltage):
        v = voltage * 1000.0
        v = int(v)
        v = v & 0x7ff8
        self.write_register(self.MAX_CHARGE_VOLTAGE_REG, v)

    def getMaxChargeVoltage(self):
        v = self.read_register(self.MAX_CHARGE_VOLTAGE_REG)
        v = v & 0x7ff8
        v = v / 1000.0
        return v

    def setChargeCurrent(self, current):
        v = current * 1000.0
        v = int(v)
        v = v & 0x1fc0
        self.write_register(self.CHARGE_CURRENT_REG, v)

    def getChargeCurrent(self):
        v = self.read_register(self.CHARGE_CURRENT_REG)
        v = v & 0x1fc0
        v = v / 1000.0
        return v

    def setMinSystemVoltage(self, voltage):
        v = voltage * 1000.0
        v = int(v)
        v = v & 0x3f00
        self.write_register(0x3e, v)

    def getMinSystemVoltage(self):
        v = self.read_register(0x3e)
        v = v & 0x3f00
        v = v / 1000.0
        return v

    def setInputCurrent(self, current):
        v = current * 1000.0 / 50.0
        v = int(v) & 0x7f
        v = v << 8
        self.write_register(0x3f, v)

    def getInputCurrent(self):
        v = self.read_register(0x3f)
        v = (v >> 8) & 0x7f
        v = v * 50.0 / 1000.0
        return v

    def setInputVoltage(self, voltage):
        v = voltage * 1000.0
        v = int(v)
        v = v & 0x3fc0
        self.write_register(0x3d, v)

    def getInputVoltage(self):
        v = self.read_register(0x3d)
        v = v & 0x3fc0
        v = v / 1000.0
        return v

    def getADCBatteryVoltage(self):
        v = self.read_register(0x26)
        v = v & 0xff
        v = (v * 0.064) + 2.88
        return v 

    def getADCSystemVoltage(self):
        v = self.read_register(0x26) >> 8
        v = v & 0xff
        v = (v * 0.064) + 2.88
        return v

    def getADCChargeCurrent(self):
        v = self.read_register(0x24) >> 8
        v = v & 0x7f
        v = v * 0.064
        return v

    def getADCDischargeCurrent(self):
        v = self.read_register(0x24) & 0x7f
        v = v * 0.256
        return v

    def getADCInputCurrent(self):
        v = self.read_register(0x25) >> 8
        v = v & 0xff
        v = v * 0.025
        return v 

    def getADCInputVoltage(self):
        v = self.read_register(0x23) >> 8
        v = v & 0xff
        v = (v * 0.064) + 3.2
        return v 