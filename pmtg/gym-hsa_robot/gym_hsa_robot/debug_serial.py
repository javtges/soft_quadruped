import serial


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
port_id = '/dev/ttyACM0'
baud = 115200
command = COMMAND_ECHO
data = blank_message


teensy = serial.Serial(port_id, baudrate=baud)
frame = START_BYTE
frame = frame + command + data
frame = frame + bytearray.fromhex("00")  # add checksum
frame = frame + END_BYTE

# print("frame", frame)
teensy.write("hi".encode())

print([True, teensy.read(20)])
