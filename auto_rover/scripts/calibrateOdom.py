def start():
	
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
		
		current_time = rospy.get_time()
		
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