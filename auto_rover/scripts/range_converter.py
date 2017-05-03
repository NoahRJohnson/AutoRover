#!/usr/bin/env python
import sys
import rospy
import math
from std_msgs.msg import Int8, UInt16
from sensor_msgs.msg import Range
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf

NODE_NAME = "range_converter"

PING_TIME_TOPIC = "/ping/timeUS"
PING_ANGLE_TOPIC = "/ping/angleDeg"
RANGE_TOPIC = "/range"

PUB_QUEUE_SIZE = 10

'''
Data is an unsigned 16 bit integer, which stores the total ultrasonic echo time
for the most recent ultrasonic sensor reading. We use the ambient air temperature
to calculate how many meters away is the nearest object based on this reading. Then
we pack this value into a Range message and send it back out. Other nodes will need
to use the ultrasound -> base_link transform to interpret this range reading.
'''
def pingTime_cb(data):

	global US_ROUNDTRIP_M, rangePub

	# Convert ping time in micro seconds to distance
	# in meters
	uS = data.data # ping time in micro seconds
	if (uS == 0):
		dist_meters = 0
	else:
		dist_meters = uS / US_ROUNDTRIP_M

	# Create and fill in Range message
	#http://www.ros.org/doc/api/sensor_msgs/html/msg/Range.html
	msg = Range()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = ULTRASOUND_FRAME_ID
	msg.radiation_type = Range.ULTRASOUND
	msg.field_of_view = 0.7 # roughly 0.7 radians arc based on PING))) datasheet
	msg.min_range = 0.02 # 2 cm
	msg.max_range = 3.0 # 3 meters
	msg.range = float(data.data)
  	
	# Publish range message
	rangePub.publish(msg)


'''
The ultrasonic sensor is mounted to a servo which pans back and forth. The Arduino
node reports the angle from the x-axis, CCW positive, that the sensor was when it
took its most recent distance measurement. Here we take that angle, convert it
from degrees to radians, create a rotation quaternion representing the yaw rotation,
and broadcast the appropriate transform between the base_link frame and the 
ultrasound frame, using that rotation quaternion and the given positional translation
between frames.
'''
def pingAngle_cb(data):

	global ultrasoundBroadcaster, ULTRASOUND_TRANSLATION

	# Convert from deg to radians
	# follows REP-103
	# http://www.ros.org/reps/rep-0103.html
	z_rotation_radians = ((data.data * math.pi) / 180) 

	# Broadcast transform from ultrasound -> base_link 
	# based on servo position and static translation
	t = TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = BASE_LINK_FRAME_ID
	t.child_frame_id = ULTRASOUND_FRAME_ID

	t.transform.translation.x = ULTRASOUND_TRANSLATION[0]
	t.transform.translation.y = ULTRASOUND_TRANSLATION[1]
	t.transform.translation.z = ULTRASOUND_TRANSLATION[2]

	q = tf.transformations.quaternion_from_euler(0, 0, z_rotation_radians) # roll, pitch, yaw
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	ultrasoundBroadcaster.sendTransform(t) 

'''
Main loop
'''
def convert_loop():

	global rangePub, ultrasoundBroadcaster

	# Setup this node with ROS Master
    	rospy.init_node(NODE_NAME) 

	# Create publishers
	rangePub = rospy.Publisher(RANGE_TOPIC, Range, queue_size=PUB_QUEUE_SIZE)

	# Create broadcaster
	ultrasoundBroadcaster = tf2_ros.TransformBroadcaster()	

	# Link callback functions to subscribed topics
	rospy.Subscriber(PING_TIME_TOPIC, UInt16, pingTime_cb)
	rospy.Subscriber(PING_ANGLE_TOPIC, Int8, pingAngle_cb)
    		
	# Listen for callbacks
	rospy.spin()

	
if __name__ == '__main__':

	global BASE_LINK_FRAME_ID, ULTRASOUND_FRAME_ID, US_ROUNDTRIP_M, ULTRASOUND_TRANSLATION
	# Grab ros params
	BASE_LINK_FRAME_ID = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
	ULTRASOUND_FRAME_ID = rospy.get_param('~ultrasound_frame_id', 'ultrasound') # the name of the ultrasound sensor's frame

	if rospy.has_param('~ambient_air_temperature_farenheit'):
		ambient_temp_F = rospy.get_param('~ambient_air_temperature_farenheit')
	else:
		ambient_temp_F = 80
		print "WARN: ros param ambient_air_temperature_farenheit missing, using default value of 80"
	if rospy.has_param('~ultrasound_frame_translation'):
		ultrasound_translation_str = rospy.get_param('~ultrasound_frame_translation')
	else:
		ultrasound_translation_str = [0.060325, 0, 0]
		print "WARN: ros param ultrasound_frame_translation missing, using default value of [0.060325 0 0]"
	
	# Try to convert param values from string
	try:
		ambient_temp_F = float(ambient_temp_F)
	except ValueError:
		print "Bad ros param ambient_air_temperature_farenheit, should be a float"
		sys.exit(2)
	try:
		ULTRASOUND_TRANSLATION = [float(x) for x in ultrasound_translation_str] # [X, Y, Z] translation values for ultrasound -> base_link
	except ValueError:
		print "Bad ros param ultrasound_frame_translation, should be 'x_translation y_translation z_translation'"
		sys.exit(2)

	# Calculate speed of sound in the air surrounding the rover, adjusting for the current air temperature. Uses formula described in: https://www.parallax.com/sites/default/files/downloads/28015-PING-Sensor-Product-Guide-v2.0.pdf
	ambient_temp_C = (ambient_temp_F - 32) * (5.0/9)
	US_ROUNDTRIP_M = 1000000 / (((ambient_temp_C * 0.6) + 331.5) * 2) # number of uS it takes sound to travel round-trip 1m (2m total), adjusted for ambient air temp

	# Run main loop
	try:
        	convert_loop()
    	except rospy.ROSInterruptException:
		pass

