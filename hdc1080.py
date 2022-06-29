
from machine import I2C
from machine import Pin
#constants

# I2C Address
HDC1080_ADDRESS =                       (0x40)    # 1000000 
# Registers
HDC1080_TEMPERATURE_REGISTER =          (0x00)
HDC1080_HUMIDITY_REGISTER =             (0x01)
HDC1080_CONFIGURATION_REGISTER =        (0x02)
HDC1080_MANUFACTURERID_REGISTER =       (0xFE)
HDC1080_DEVICEID_REGISTER =         (0xFF)
HDC1080_SERIALIDHIGH_REGISTER =         (0xFB)
HDC1080_SERIALIDMID_REGISTER =          (0xFC)
HDC1080_SERIALIDBOTTOM_REGISTER =       (0xFD)



#Configuration Register Bits

HDC1080_CONFIG_RESET_BIT =              (0x8000)
HDC1080_CONFIG_HEATER_ENABLE =          (0x2000)
HDC1080_CONFIG_ACQUISITION_MODE =       (0x1000)
HDC1080_CONFIG_BATTERY_STATUS =         (0x0800)
HDC1080_CONFIG_TEMPERATURE_RESOLUTION = (0x0400)
HDC1080_CONFIG_HUMIDITY_RESOLUTION_HBIT =    (0x0200)
HDC1080_CONFIG_HUMIDITY_RESOLUTION_LBIT =    (0x0100)

HDC1080_CONFIG_TEMPERATURE_RESOLUTION_14BIT = (0x0000)
HDC1080_CONFIG_TEMPERATURE_RESOLUTION_11BIT = (0x0400)

HDC1080_CONFIG_HUMIDITY_RESOLUTION_14BIT = (0x0000)
HDC1080_CONFIG_HUMIDITY_RESOLUTION_11BIT = (0x0100)
HDC1080_CONFIG_HUMIDITY_RESOLUTION_8BIT = (0x0200)

I2C_SLAVE=0x0703

import struct, array, time, io

sda_pin = Pin(33)
scl_pin = Pin(32)
i2c_dev = None;


class HDC1080:
    def __init__(self, twi=1, addr=HDC1080_ADDRESS ):
       global sda_pin, scl_pin
       global i2c_dev

       i2c_dev = I2C(0, scl = scl_pin, sda = sda_pin, freq = 100000)
       
       time.sleep(0.015) # 15ms startup time

       config = HDC1080_CONFIG_ACQUISITION_MODE 

       self.writeData(HDC1080_CONFIGURATION_REGISTER, config)
       time.sleep(0.015) # From the data sheet
            
       # 0x10(48)    Temperature, Humidity enabled, Resolultion = 14-bits, Heater off
       #config = HDC1080_CONFIG_ACQUISITION_MODE 
       #self.writeData(HDC1080_CONFIGURATION_REGISTER, config)

    # public functions

    def readTemperature(self):
        temp = self.readData(HDC1080_TEMPERATURE_REGISTER)
        cTemp = (temp / 65536.0) * 165.0 - 40
        return cTemp


    def readHumidity(self):
        # Send humidity measurement command, 0x01(01)
        humidity = self.readData(HDC1080_HUMIDITY_REGISTER)
        humidity = (humidity / 65536.0) * 100.0
        return humidity

    def readConfigRegister(self):
        # Read config register
        return self.readData(HDC1080_CONFIGURATION_REGISTER)


    def turnHeaterOn(self):
        # Read config register
        config = self.readConfigRegister()
        config = config | HDC1080_CONFIG_HEATER_ENABLE 
        self.writeData(HDC1080_CONFIGURATION_REGISTER, config)
        return

    def turnHeaterOff(self):
        # Read config register
        config = self.readConfigRegister()
        config = config & ~HDC1080_CONFIG_HEATER_ENABLE 
        self.writeData(HDC1080_CONFIGURATION_REGISTER, config)
        return



    def setHumidityResolution(self,resolution):
        # Read config register
        config = self.readConfigRegister()
        config = (config & ~0x0300) | resolution 
        self.writeData(HDC1080_CONFIGURATION_REGISTER, config)
        return

    def setTemperatureResolution(self,resolution):
        # Read config register
        config = self.readConfigRegister()
        config = (config & ~0x0400) | resolution 
        self.writeData(HDC1080_CONFIGURATION_REGISTER, config)
        return

    def readBatteryStatus(self):
        # Read config register
        config = self.readConfigRegister()
        config = config & ~ HDC1080_CONFIG_HEATER_ENABLE

        if (config == 0):
            return True
        else:
            return False


    def readManufacturerID(self):
        return self.readData(HDC1080_MANUFACTURERID_REGISTER)

    def readDeviceID(self):
        return self.readData(HDC1080_DEVICEID_REGISTER)

    def readSerialNumber(self):
        serialNumber = self.readData(HDC1080_SERIALIDHIGH_REGISTER)

        temp =  self.readData(HDC1080_SERIALIDMID_REGISTER);
        serialNumber = (serialNumber << 16) + temp 

        temp = self.readData(HDC1080_SERIALIDBOTTOM_REGISTER)
        serialNumber = (serialNumber << 16) + temp 

        return serialNumber

    def readData(self, pointer):
        s2 = bytearray([pointer])
        i2c_dev.writeto(HDC1080_ADDRESS, s2)
        time.sleep(0.0625)
        data = i2c_dev.readfrom(HDC1080_ADDRESS, 2) # read 2 bytes
        return (data[0] << 8) + data[1]

    def writeData(self, pointer, value):
        s = [pointer, value>>8, value & 0xff]
        s2 = bytearray( s )
        i2c_dev.writeto(HDC1080_ADDRESS, s2 ) #sending config register bytes
        time.sleep(0.015)               # From the data sheet