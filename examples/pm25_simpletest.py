# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
Example sketch to connect to PM2.5 sensor with either I2C or UART.
"""

# pylint: disable=unused-import
import time
import board
import busio
from digitalio import DigitalInOut, Direction, Pull
from adafruit_pm25.i2c import PM25_I2C
import pandas as pd


reset_pin = None
# If you have a GPIO, its not a bad idea to connect it to the RESET pin
# reset_pin = DigitalInOut(board.G0)
# reset_pin.direction = Direction.OUTPUT
# reset_pin.value = False


# For use with a computer running Windows:
# import serial
# uart = serial.Serial("COM30", baudrate=9600, timeout=1)

# For use with microcontroller board:
# (Connect the sensor TX pin to the board/computer RX pin)
# uart = busio.UART(board.TX, board.RX, baudrate=9600)

#For use with Raspberry Pi/Linux:
import serial
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=0.25)

# For use with USB-to-serial cable:
# import serial
# uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=0.25)

# Connect to a PM2.5 sensor over UART
from adafruit_pm25.uart import PM25_UART
pm25 = PM25_UART(uart, reset_pin)

x = time.time()
y = 0
# Create library object, use 'slow' 100KHz frequency!
#i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
# Connect to a PM2.5 sensor over I2C
#pm25 = PM25_I2C(i2c, reset_pin)

print("Found PM2.5 sensor, reading data...")
pm10 = []
pm2 = []
pm100 = []
timeLoop = []
while y-x<=5:
    y = time.time()
    try:
        timeLoop.append(y)
        aqdata = pm25.read()
        pm10.append(aqdata["pm10 standard"])
        pm2.append(aqdata["pm25 standard"])
        pm100.append(aqdata["pm100 standard"])
        

        #print(aqdata)
    except RuntimeError:
        print("Unable to read from sensor, retrying...")
        continue
    
    time.sleep(1)

 

dict= {"Time": timeLoop ,"PM 1.0":pm10 ,  "PM 2.5":pm2 ,"PM10":pm100 }
df = pd.DataFrame(dict)
df.to_csv('aqdata.csv')


   
