import serial
import codecs
import sys
import time
import numpy as np

COMMAND_ECHO = bytearray.fromhex("62")
COMMAND_PING = bytearray.fromhex("61")
COMMAND_SERVO = bytearray.fromhex("63")
COMMAND_LED = bytearray.fromhex("64")
START_BYTE = bytearray.fromhex("41")
END_BYTE = bytearray.fromhex("5A")


port = '/dev/ttyACM0'
baud = 115200

blank_message = bytearray.fromhex("00000000000000000000000000000000")

# This function composes and transmits each command frame. 
# It must be provided with the command byte, data payload,
# and length of expected reply (if appropriate)
# This function will return a list in the form [bool, responce] where 
# bool is True if the communication was succesful, and False if it failed. 
# if a length of 0 is specified for the responce that index will be set at ""
def send_frame(command, data, reply_length = 0, port_id = None):
	
	# if no port has been provided use the global default. 
	if port_id == None: 
		port_id = port

	try: 
	    teensy = serial.Serial(port_id,baudrate = baud)
	    frame = START_BYTE
	    frame = frame + command + data
	    frame = frame + bytearray.fromhex("00") # add checksum
	    frame = frame + END_BYTE
	    
	    print("frame", frame)
	    teensy.write(frame)
	    if reply_length != 0:
	        return [True, teensy.read(reply_length)]
	    else:
	    	return [True, ""]
	except serial.SerialException as e: 
		print("serial exception: " + str(e))
		return[False,""]

def request_echo(payload):
    responce = send_frame(COMMAND_ECHO, payload, 2)
    return responce

def set_light(lit, port_id = None):
    message =  blank_message
    message[0] = lit
    send_frame(COMMAND_LED, message, port_id = port_id)

# Requests the device ID of the target device. 
def request_ping(port_id = None):
	responce = send_frame(COMMAND_PING,
                              blank_message,
                              reply_length = 3,
                              port_id = port_id)
	return responce

def set_servos(angles, port_id = None):
    message = blank_message
    for i in range(0, 16):
        assert(angles[i] < 256 and angles[i] >= 0)
        message[i] =  angles[i]
    send_frame(COMMAND_SERVO, message, port_id = port_id)   

# <<<<<<< HEAD
set_light(True)
time.sleep(2)
set_light(False)
print(request_ping())
time.sleep(1)
set_light(False)
time.sleep(2)
#servo_positions = np.ones(16, 'uint8')*255
#set_servos(servo_positions)
#set_light(True)
#time.sleep(1)
#print(request_ping())
#set_light(False)
#time.sleep(1)
servo_positions = np.ones(16, 'uint8')*255


'''
time.sleep(10)

for i in range(100):
	for j in range(16):
		if i%2 == 0:
			servo_positions[j] += 10
		else:
			servo_positions[j] -= 10
		set_servos(servo_positions)
		time.sleep(.2)
'''
request_ping()
