# Importing Libraries

'''
Tests serial communication with Arduino.
'''

import serial
import time
arduino = serial.Serial('/dev/ttyACM0',115200,timeout=0)
def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data
while True:
    num = input("Enter a number: ") # Taking input from user
    value = write_read(num)
    print(value) # printing the value
    

# From: https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0