# This file is executed on every boot (including wake-boot from deepsleep)
import utime
from random import randint
import machine
from machine import Pin
import network
from umqtt.simple import MQTTClient
import esp
import _thread
import json
import random
import ubinascii

import tca6416
import hdc1080
import bq25710

import config
from machine import WDT

import uasyncio
from machine import UART

wdt = WDT(timeout=30000) # enable it with a timeout of 2s
random.seed();

with open(config.KEYFILE, 'r') as f:
    key = f.read()

with open(config.CERTFILE, 'r') as f:
    cert = f.read()

# SSL certificates.
SSL_PARAMS = {'key': key,'cert': cert, 'server_side': False}

##### INIT HARDWARE ######
#uart = UART(1, 9600)
#uart.init(9600, bits = 8, parity = None, stop = 1, tx = 5, rx = 13)
#uart2 = UART(2, 38400)
#uart2.init(38400, bits = 8, parity = None, stop = 1, tx = 2, rx = 15)

ups = bq25710.BQ25710()
#set charge option 1 & 2 & 3
ups.write_register(0x30, 0b1000001000000001)
ups.write_register(0x31, 0b0000001000110111)
ups.write_register(0x32, 0b0010000000110000)

#set adc
ups.write_register(0x35, 0b1110000001111111)

#set charge voltage (12.928v)
ups.write_register(0x15, 0b0011001010000000)

#set charge current (2A)
ups.write_register(0x14, 0b0000100000000000)

#set min system voltage (6.144v)
#ups.write_register(0x3e, 0b0001100000000000)
ups.setMinSystemVoltage(10.0)

#set adapter input current limit (5A)
ups.write_register(0x3f, 0b0110010000000000)

#set adapter input voltage limit (8.192v)
ups.write_register(0x3d, 0b0010000000000000)

#set charge option 0
ups.write_register(0x12, 0b0000001000001100)

hdc = hdc1080.HDC1080()

gpio_exp = tca6416.TCA6416()
gpio_exp.pin_mode(tca6416.GPIO_Pin.P1_7, tca6416.Mode.INPUT) #input4
gpio_exp.pin_mode(tca6416.GPIO_Pin.P1_6, tca6416.Mode.INPUT) #input3
gpio_exp.pin_mode(tca6416.GPIO_Pin.P1_5, tca6416.Mode.INPUT) #input2
gpio_exp.pin_mode(tca6416.GPIO_Pin.P1_4, tca6416.Mode.INPUT) #input1
gpio_exp.pin_mode(tca6416.GPIO_Pin.P1_3, tca6416.Mode.OUTPUT) #relay4
gpio_exp.pin_mode(tca6416.GPIO_Pin.P1_2, tca6416.Mode.OUTPUT) #relay3
gpio_exp.pin_mode(tca6416.GPIO_Pin.P1_1, tca6416.Mode.OUTPUT) #relay2
gpio_exp.pin_mode(tca6416.GPIO_Pin.P1_0, tca6416.Mode.OUTPUT) #relay1

gpio_exp.pin_mode(tca6416.GPIO_Pin.P0_5, tca6416.Mode.OUTPUT) #led_user

gpio_exp.digital_write(tca6416.GPIO_Pin.P1_0, False)
gpio_exp.digital_write(tca6416.GPIO_Pin.P1_1, False)
gpio_exp.digital_write(tca6416.GPIO_Pin.P1_2, False)
gpio_exp.digital_write(tca6416.GPIO_Pin.P1_3, False)

led_state = False
gpio_exp.digital_write(tca6416.GPIO_Pin.P0_5, led_state)

topic_setrelays = b'device/ups/{}/setrelays'.format(config.CLIENT_ID)

topic_relays = b'device/ups/{}/statusrelays'.format(config.CLIENT_ID)
topic_config = b'device/ups/{}/config'.format(config.CLIENT_ID)
topic_status = b'device/ups/{}/status'.format(config.CLIENT_ID)
topic_request = b'device/ups/{}/request'.format(config.CLIENT_ID)
topic_airquality = b'device/ups/{}/airquality'.format(config.CLIENT_ID)
topic_qrreader = b'device/ups/{}/qrreader'.format(config.CLIENT_ID)

topic_sensor = b'device/ups/{}/sensor'.format(config.CLIENT_ID)
topic_battery = b'device/ups/{}/battery'.format(config.CLIENT_ID)
topic_inputs = b'device/ups/{}/inputs'.format(config.CLIENT_ID)

###########################

def reboot():
  machine.reset()

def webpython():
  webrepl.start()

def no_debug():
  esp.osdebug(None)

class wireless_network:
  def __init__(self):
    self.sta_if = None
  def connect(self, ssid, password):
    self.sta_if = network.WLAN(network.STA_IF)
    self.sta_if.active(False)
    if not self.sta_if.isconnected():
      self.sta_if.active(True)
      self.sta_if.connect(ssid, password)
      while not self.sta_if.isconnected(): 
        pass
  def status(self):
    print('network config:', self.sta_if.ifconfig())
  def isconnected(self):
    return self.sta_if.isconnected()


def client_callback(topic, msg):
  print(topic, msg)
  if topic == topic_setrelays:
    try:
      js = json.loads(msg)
      if 'relay1' in js:
        gpio_exp.digital_write(tca6416.GPIO_Pin.P1_0, js['relay1'])
      if 'relay2' in js:
        gpio_exp.digital_write(tca6416.GPIO_Pin.P1_1, js['relay2'])
      if 'relay3' in js:
        gpio_exp.digital_write(tca6416.GPIO_Pin.P1_2, js['relay3'])
      if 'relay4' in js:
        gpio_exp.digital_write(tca6416.GPIO_Pin.P1_3, js['relay4'])
    except Exception as e:
      print("Callback error:")
      print(e)
      pass

async def led_blink_task():
  while True:
    gpio_exp.digital_write(tca6416.GPIO_Pin.P0_5, True)
    await uasyncio.sleep_ms(500)
    gpio_exp.digital_write(tca6416.GPIO_Pin.P0_5, False)
    await uasyncio.sleep_ms(500)

async def uart_publish_task(client):
  uart = UART(1)
  uart.init(9600, bits = 8, parity = None, stop = 1, tx = 5, rx = 13)
  while True:
    uart.write(b'\x01\x03\x00\x10\x00\x08\x45\xc9')
    await uasyncio.sleep_ms(500)
    if uart.any():
      await uasyncio.sleep_ms(500)
      data = uart.read()
      co2 = data[3]*256 + data[4]
      ch2o = data[5]*256 + data[6]
      tvoc = data[7]*256 + data[8]
      pm1_0 = data[9]*256 + data[10]
      pm2_5 = data[11]*256 + data[12]
      pm10_0 = data[13]*256 + data[14]
      temperature = float(data[15] * 10 + data[16]) / 10.0
      humidity = float(data[17] * 10 + data[18]) / 10.0
      airDict = {}
      airDict['CO2'] = co2
      airDict['CH2O'] = ch2o
      airDict['TVOC'] = tvoc
      airDict['PM1.0'] = pm1_0
      airDict['PM2.5'] = pm2_5
      airDict['PM10'] = pm10_0
      airDict['Temperature'] = temperature
      airDict['Humidity'] = humidity
      client.publish(topic_airquality, json.dumps(airDict))
    await uasyncio.sleep_ms(1000)

async def uart2_publish_task(client):
  uart2 = UART(2, 38400)
  uart2.init(38400, bits = 8, parity = None, stop = 1, tx = 2, rx = 15)
  while True:
    if uart2.any():
      await uasyncio.sleep(1)
      client.publish(topic_qrreader, uart2.read())
    await uasyncio.sleep_ms(0)


async def publish_task(client):
  global ups
  global gpio_exp
  global hdc
  
  mac = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
  while True:
    
    sensorDict = {}
    sensorDict['Id'] = mac
    sensorDict['Temperature'] = float(hdc.readTemperature());
    sensorDict['Humidity'] = float(hdc.readHumidity());

    client.publish(topic_sensor, json.dumps(sensorDict))

    del(sensorDict)

    statusDict = {}
    statusDict['Id'] = mac
    statusDict['VoltageIn'] = float(ups.getADCInputVoltage())
    statusDict['Current'] = float(ups.getADCInputCurrent())
    statusDict['VoltageOut'] = float(ups.getADCSystemVoltage())

    client.publish(topic_status, json.dumps(statusDict))

    del(statusDict)

    batteryDict = {}
    batteryDict['Id'] = mac
    batteryDict['Charging'] = bool((ups.read_register(0x20) & (1 << 10)) > 0) #IN_FCHRG bit
    batteryDict['Voltage'] = float(ups.getADCBatteryVoltage())
    batteryDict['ChargeCurr'] = float(ups.getADCChargeCurrent())
    batteryDict['DischargeCurr'] = float(ups.getADCDischargeCurrent())
    batteryDict['VoltageOut'] = 12.0
    batteryDict['Temperature'] = 25.0

    client.publish(topic_battery, json.dumps(batteryDict))

    del(batteryDict)

    relayDict = {}
    relayDict['Id'] = mac
    relayDict['relay1'] = bool(gpio_exp.digital_read(tca6416.GPIO_Pin.P1_0))
    relayDict['relay2'] = bool(gpio_exp.digital_read(tca6416.GPIO_Pin.P1_1))
    relayDict['relay3'] = bool(gpio_exp.digital_read(tca6416.GPIO_Pin.P1_2))
    relayDict['relay4'] = bool(gpio_exp.digital_read(tca6416.GPIO_Pin.P1_3))

    client.publish(topic_relays, json.dumps(relayDict))

    del(relayDict)

    inputsDict = {}
    inputsDict['Id'] = mac
    inputsDict['input1'] = bool(gpio_exp.digital_read(tca6416.GPIO_Pin.P1_4))
    inputsDict['input2'] = bool(gpio_exp.digital_read(tca6416.GPIO_Pin.P1_5))
    inputsDict['input3'] = bool(gpio_exp.digital_read(tca6416.GPIO_Pin.P1_6))
    inputsDict['input4'] = bool(gpio_exp.digital_read(tca6416.GPIO_Pin.P1_7))

    client.publish(topic_inputs, json.dumps(inputsDict))

    del(inputsDict)

    gc.collect()

    await uasyncio.sleep(1)


async def mythread():
  print("Start thread...")
  try:
    print('connecting to network...')
    wifi = wireless_network()
    wifi.connect(config.WIFI_SSID, config.WIFI_PASSWORD)
    wifi.status()

    gc.collect()
    client =  MQTTClient( config.CLIENT_ID, config.AWS_ENDPOINT, port = 8883, keepalive = 10000, ssl = True, ssl_params = SSL_PARAMS )
    client.set_callback(client_callback)
    gc.collect()
    client.connect()
    #client.subscribe(topic_status)
    client.subscribe(topic_setrelays)
    print('connected to mqtt client.')
    
    global ups
    global gpio_exp
    global hdc

    # loop executes once a second
    # but sensor reading only taken every INTERVAL seconds
    # (this is for timely response of subscribed topics)
    uasyncio.create_task(led_blink_task())
    uasyncio.create_task(publish_task(client))
    uasyncio.create_task(uart_publish_task(client))
    uasyncio.create_task(uart2_publish_task(client))

    while True:
      # check subscriptions
      client.check_msg()
      await uasyncio.sleep_ms(0)
      wdt.feed()

  except Exception as e:
    print("Error:")
    print(e)

  print("End thread")
  await uasyncio.sleep(1)
  gc.collect()

try:
  retrys = 0
  while(True):
    #uasyncio.create_task(task_led())
    uasyncio.run(mythread())
    uasyncio.new_event_loop()
    gc.collect()
    retrys += 1
    if retrys > 10:
      machine.reset()
except Exception as e:
  print(e)
finally:
  uasyncio.new_event_loop()
  gc.collect()
machine.reset()