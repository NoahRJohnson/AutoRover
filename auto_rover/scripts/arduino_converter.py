#!/usr/bin/env python
import sys
import rospy
import math
from std_msgs.msg import Int8, UInt16
from auto_rover.msg import EncCount
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

NODE_NAME = "arduino_converter"

ENCODER_TICKS_TOPIC = "/odom/encTicks"
PING_TIME_TOPIC = "/ping/timeUS"
PING_ANGLE_TOPIC = "/ping/angleDeg"
RANGE_TOPIC = "/range"
ODOM_TOPIC = "/odom/raw"

ULTRASOUND_FRAME_ID = "ultrasound"
BASE_LINK_FRAME_ID = "base_link"
ODOM_FRAME_ID = "odom"

# declare globals
US_ROUNDTRIP_M = 0 # number of uS it takes sound to travel round-trip 1m (2m total), adjusted for ambient air temp
ULTRASOUND_TRANSLATION = 0 # [X, Y, Z] translation values for ultrasound -> base_link
odomPub = 0 # Odometry publisher
rangePub = 0 # Range publisher
tf_broadcaster = 0 # Transform broadcaster
rover_pose_x = 0
rover_pose_y = 0
rover_pose_th = 0
rover_velocity_x = 0
rover_velocity_y = 0
rover_velocity_th = 0

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
	if (uS == 0)
		dist_meters = 0;
	else
		dist_meters = uS / US_ROUNDTRIP_M;

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
and broadcast the appropriate transform between the ultrasound frame and the 
base_link frame, using that rotation quaternion and the given positional translation
between frames.
'''
def pingAngle_cb(data):

	global tf_broadcaster, ULTRASOUND_TRANSLATION

	# Convert from deg to radians
	z_rotation_radians = ((data.data * math.pi) / 180) 

	# Broadcast transform from ultrasound -> base_link 
	# based on servo position and static translation
	tf_broadcaster.sendTransform(ULTRASOUND_TRANSLATION,
		tf.transformations.quaternion_from_euler(0, 0, z_rotation_radians), # roll, pitch, yaw
		rospy.Time.now(),
		BASE_LINK_FRAME_ID,
		ULTRASOUND_FRAME_ID)


'''
EncCount is a custom message type which simply stores a snapshot of encoder
tick counts for the left and right wheels at a time stamp. We use a simple
rolling average with the kinematics equations for a differential drive
robot (an approximation of our skid-steering rover), where the two imaginary
wheels on both side of the rover are assumed to 'exist' at an average position
of the two actual wheels. This average then gives us an estimate of the
position and velocity of our rover, which we publish in an Odometry message.
This odometry message is essentially dead reckoning, and is one of the inputs
fused into the state estimation node. 
'''
def encCount_cb(data):
	global tf_broadcaster

	# Pick out data from EncCount message
	leftTicks = data.leftTicks
	rightTicks = data.rightTicks
	timeStamp = data.stamp

	# Create odometry message
	msg = Odometry()
	msg.header.stamp = rospy.Time(timeStamp.secs, timeStamp.nsecs) # copy data timestamp
	msg.header.frame_id = ODOM_FRAME_ID # Coordinate frame of pose
	msg.child_frame_id = BASE_LINK_FRAME_ID # Coordinate frame of twist

	# Fill in Pose
# positive ticks move in the CW Direction
	msg.pose.pose.position = Point()
	msg.pose.pose.orientation = Quaternion()

	msg.pose.covariance

	# Fill in twist
	
	# Publish odometry message
	odomPub.publish(msg)


'''
Main loop
'''
def convert_loop(airTemp, ultrasound_translation_list):

	global odomPub, rangePub, ultrasoundBroadcaster

	# Setup this node with ROS Master
    	rospy.init_node(NODE_NAME) 

	# Create publishers
	odomPub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=PUB_QUEUE_SIZE)
	rangePub = rospy.Publisher(RANGE_TOPIC, Range, queue_size=PUB_QUEUE_SIZE)

	# Create broadcaster
	ultrasoundBroadcaster = tf.TransformBroadcaster()	

	# Link callback functions to subscribed topics
	rospy.Subscriber(ENCODER_TICKS_TOPIC, EncCount, encCount_cb)
	rospy.Subscriber(PING_TIME_TOPIC, UInt16, pingTime_cb)
	rospy.Subscriber(PING_ANGLE_TOPIC, Int8, pingAngle_cb)
    		
	# Listen for callbacks
	rospy.spin()

	
if __name__ == '__main__':

	# Grab ros params
	if rospy.has_param('~ambient_air_temperature_farenheit'):
		ambient_temp_F = rospy.get_param('~ambient_air_temperature_farenheit')
	else:
		ambient_temp_F = 80
		print "WARN: ros param ambient_air_temperature_farenheit missing, using default value of 80"
	if rospy.has_param('~ultrasound_frame_translation'):
		ultrasound_translation_str = rospy.get_param('~ultrasound_frame_translation')
	else:
		ultrasound_translation_str = [0.060325 0 0]
		print "WARN: ros param ultrasound_frame_translation missing, using default value of [0.060325 0 0]"
	
	# Try to convert param values from string
	try:
		ambient_temp_F = float(ambient_temp_F)
	except ValueError:
		print "Bad ros param ambient_air_temperature_farenheit, should be a float"
		sys.exit(2)
	try:
		ULTRASOUND_TRANSLATION = [float(x) for x in ultrasound_translation_str.split()]
	except ValueError:
		print "Bad ros param ultrasound_frame_translation, should be 'x_translation y_translation z_translation'"
		sys.exit(2)

	# Calculate speed of sound in the air surrounding the rover, adjusting for the current air temperature. Uses formula described in: https://www.parallax.com/sites/default/files/downloads/28015-PING-Sensor-Product-Guide-v2.0.pdf
	ambient_temp_C = (ambient_temp_F - 32) * (5.0/9)
	US_ROUNDTRIP_M = 1000000 / ((ambient_temp_C * 0.6) + 331.5) * 2); #global

	# Run main loop
	try:
        	convert_loop()
    	except rospy.ROSInterruptException:
		pass

