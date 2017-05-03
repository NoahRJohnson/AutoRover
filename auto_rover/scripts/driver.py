'''
This driver reads motor speeds in degrees from 0-180 (44 full forward, 94 stop, 144 full reverse) from the command line and outputs ros messages containing the same data to the /cmd/left and /cmd/right topics which the Arduino node is subscribed to, and will set the motors speed to.
'''

#!/usr/bin/env python
import sys
import rospy
import time

CMD_LEFT_MOTORS_TOPIC = "/cmd/left"
CMD_RIGHT_MOTORS_TOPIC = "/cmd/right"

while (True):
	output_bytes = bytearray()

	direction = raw_input("Direction (L/R): ")
	if direction.upper() not in ['L','R']:
		print "invalid direction"
		continue
	else:
		output_bytes.extend(direction.upper())

	speed = raw_input("Speed (0 - 180): ")
	try:
		speed = int(speed)
		if speed < 0 or speed > 180:
			raise Exception('invalid speed')
	except:
		print "invalid speed"
		continue

	# first generate an ASCII encoding of the
	# hex representation of the integer iData
	ascii_string = hex(speed)[2:] 
	
	# then encode that string into an array
	# of bytes in ascii
	output_bytes.extend(ascii_string)

	ser.write(output_bytes)

	ser.flush()
	
ser.close()

def drive(portName):
	
	float US_ROUNDTRIP_M;

def usage():
	print "Usage: python drive.py [port_name]"

if __name__ == '__main__':
	rospy.get_param('~private_param_name')
	rospy.get_param('~private_param_name')
	if len(sys.argv) < 1:
		usage()
	else:
		portName = sys.argv[1]

    		try:
        		drive(portName)
    		except rospy.ROSInterruptException:
        		pass


