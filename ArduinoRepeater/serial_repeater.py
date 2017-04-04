#!/usr/bin/env python
import sys
import rospy
import serial
from nav_msgs.msg import Odometry

# parameters specific to arduino serial comm
PORT_NAME = 'COM1'
BAUDRATE = 9600
NUM_PARITY = serial.PARITY_NONE
NUM_STOPBITS = serial.STOPBITS_ONE
NUM_DATABITS = serial.EIGHTBITS\

NODE_NAME = "Serial_Repeater"
ODOM_TOPIC = "raw_odom"
RANGE_TOPIC = ""




print("connected to: " + ser.portstr)

#this will store the line
line = []

while True:
    for c in ser.read():
        line.append(c)
        if c == '\n':
            print("Line: " + line)
            line = []
            break



def run_repeater_node(distancePerTick, lenBetweenWheels):

	# timeout = None -> read() blocks forever until requested number
	# of bytes are received.
	# port is immediately opened on object creation
	ser = serial.Serial(\
		port=PORT_NAME,\
		baudrate=BAUDRATE,\
		parity=NUM_PARITY,\
		stopbits=NUM_STOPBITS,\
		bytesize=NUM_DATABITS,\
			timeout=None)
		
    odom_pub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=10)
    rospy.init_node(NODE_NAME)
    while not rospy.is_shutdown():
		
		current_time = rospy.get_rostime()
		
		# Should come in a stream in the following order:
		# L == Left motor's quadrature encoder's position value
		# R == Right motor's quadrature encoder's position value
		# S == Angular degree of servo
		# P == Ultrasonic ping time (in micro seconds) measured at the given servo angle
		if not readType(ser, 'L'):
			break
		bytes_hex = ser.read(8)
		enc1_count = hexASCII2Int(bytes_hex)
			
		if not readType(ser, 'R'):
			break
		bytes_hex = ser.read(8)
		enc2_count = hexASCII2Int(bytes_hex)
		
		if not readType(ser, 'S'):
			break
		bytes_hex = ser.read(2)
		servoPos = hexASCII2Int(bytes_hex)
			
		if not readType(ser, 'P'):
			break
		bytes_hex = ser.read(4)
		pingTime_uS = hexASCII2Int(bytes_hex)
		
		# Now calculate
		
		# Now fill in the odom message
		
		# Publish the odom message
		odom_pub.publish()	
		
	
	ser.close()
	
	

# translates a byte array holding hexadecimal ASCII into an integer
def hexASCII2Int(hexBytes):# prints arbitrary byte[] data in hex
	hex_str = hexBytes.decode('ascii')
	return int(hex_str, 16)

#
def readType(ser, expected_type):
	type = ser.read(1)
	if type == expected_type:
		return True
	elif (type == ""): # error
		rospy.logerr("Unable to read from serial port. Shutting down repeater node.")
	else:
		rospy.logerr("Received type char '%s', expected '%s'. Shutting down repeater node." % type.decode('ascii'), expected_type)
	return False
		

if __name__ == '__main__':
	num_args = len(sys.argv)
	args = str(sys.argv)
	
    try:
        run_repeater_node()
    except rospy.ROSInterruptException:
        pass
		
