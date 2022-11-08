'''
includes all functions necessary to set up the gait environment
'''

# imports
import serial
import numpy as np
 

### SETUP ###
# byte array
COMMAND_ECHO = bytearray.fromhex("62")
COMMAND_PING = bytearray.fromhex("61")
COMMAND_SERVO = bytearray.fromhex("63")
COMMAND_LED = bytearray.fromhex("64")
START_BYTE = bytearray.fromhex("41")
END_BYTE = bytearray.fromhex("5A")
blank_message = bytearray.fromhex("00000000000000000000000000000000")

# usb connection
port = '/dev/ttyACM0'
baud = 115200

# positions for set_servos function
servo_positions = np.zeros(16, 'uint8')


'''This function composes and transmits each command frame.
It must be provided with the command byte, data payload,
and length of expected reply (if appropriate)
This function will return a list in the form [bool, responce] where
bool is True if the communication was succesful, and False if it failed.
if a length of 0 is specified for the responce that index will be set at ""
'''
def send_frame(command, data, reply_length=0, port_id=None):
    # if no port has been provided use the global default.
    if port_id == None:
        port_id = port

    try:
        teensy = serial.Serial(port_id, baudrate=baud)
        frame = START_BYTE
        frame = frame + command + data
        frame = frame + bytearray.fromhex("00")  # add checksum
        frame = frame + END_BYTE

        # print(frame)
        teensy.write(frame)
        if reply_length != 0:
            return [True, teensy.read(reply_length)]
        else:
            return [True, ""]
    except serial.SerialException as e:
        print("serial exception: " + str(e))
        return [False, ""]


def request_echo(payload):
    responce = send_frame(COMMAND_ECHO, payload, 2)
    return responce


def set_light(lit, port_id=None):
    message = blank_message
    message[0] = lit
    send_frame(COMMAND_LED, message, port_id=port_id)


'''requests the device ID of the target device
'''
def request_ping(port_id=None):
    responce = send_frame(COMMAND_PING,
                            blank_message,
                            reply_length=3,
                            port_id=port_id)
    return responce