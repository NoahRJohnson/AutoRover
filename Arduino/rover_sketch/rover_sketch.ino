#include <Servo.h>
#include <NewPing.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <auto_rover/EncCount.h>

/************************
 * Hardware Definitions *
 ************************/
 
// Make sure that the Arduino's GND is connected to motor driver's GND

/**
 * Number of symbols that will be transferred over serial port
 * per second. Arduino serial defaults to 8N1 which uses 10 symbols
 * per byte of data (1 start symbol, 8 data bits, 1 stop symbol).
 */
#define BAUD_RATE 28800 // 2880 bytes per second, roughly 34 micro seconds between bits

/** 
 * Arduino digital pin connection definitions 
 */
#define encLeftAPin 2 // Hardware interrupt pin, channel A output
#define encLeftBPin 4 // Channel B output
#define encRightAPin 3 // Hardware interrupt pin, channel A output
#define encRightBPin 7 // Channel B output
#define rightMotorsPin 5 // S1 on Sabertooth
#define leftMotorsPin 6 // S2 on Sabertooth
#define sonicServoPin 9 // standard Parallax servo attached to ultrasonic sensor
#define TRIGGER_PIN  11  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). 3 meters is the limit of the PING))) sensor.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Setup NewPing to use the appropriate pins and settings

Servo rightMotors, leftMotors; // Sabertooth R/C differential control. 44 deg full reverse, 94 stopped, 144 deg full forward for both of them thanks to hardware cables swapped.
#define MOTOR_CENTER 94
Servo sonicServo; // standard Parallax servo attached to ultrasonic sensor. xx left, 66 center, xx 

/*
 * We use the Arduino Servo library to write pulses out to the servo telling it where to position itself.
 * We will sweep the servo back and forth to detect obstacles ahead and to the sides of the rover.
 * The servo takes degrees from 0 to 180. However, the ping sensor is not mounted exactly at the servo's 
 * center point. Thus, we will define the range that we wish the servo to sweep between.
 */
#define SERVO_LEFT 150 // Degree corresponding to ping sensor pointing left
#define SERVO_CENTER 66 // Degree corresponding to ping sensor pointing straight
#define SERVO_RIGHT 0 // Degree corresponding to ping sensor pointing right

#define SERVO_STEP_SZ 1 // How many degrees to increment or decrement servo sweep by at each step (should be a divisor of 5)
#define SERVO_STEP_DELAY 100UL // How many ms to wait in-between servo steps, and thus between pings and encoder readings (must be UL for timer compare)
#define SERVO_DEFAULT_DEG 93 // Which angular position the servo defaults to when attached but given no directions.



/**************************************************************
 * Interrupt Service Routines for the left and right encoders *
 **************************************************************/
 
 /*
 * Signed longs are 4 bytes long (about -2 billion to 2 billion).
 * There are about 680 counts per second of the rotary encoders, so
 * in theory the earliest that these longs can under- or over-flow 
 * is in (2,000,000,000 / 680) / 3600 =~ 816 hours, or 34 days of
 * constantly running the Arduino. For the purposes of this project we can
 * get away with not guarding for this error. But in a commercial project which 
 * might have month-long continuous runtimes, this error should be handled in the
 * loop() routine, using interrupt guards to check the encoder values.
 */
volatile long encLeftCount = 0L;
volatile long encRightCount = 0L;

const int8_t encoder_lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
/**
 * Credit:
 * http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
 * 
 * ISRs must be as fast as possible, so rather than clear coding practice (like using
 * (1 << encLeftAPin), I will conserve operations by using constants where I can (0b100).
 */
void encoderLeft_isr() {
  static uint8_t encLeft_val = 0;

  encLeft_val = encLeft_val << 2; // Store the previous 2-bit code

  /* 
   *  PIND returns input readings from pins 0-7 as a byte.
   *  https://www.arduino.cc/en/Reference/PortManipulation
   *  The below code uses bit shifting and masking to put
   *  the values of digital input pins 4 (output B) and 
   *  2 (output A) into the least significant bit and 
   *  second-least significant bit of the enc_val 
   *  variable, respectively.
   */
  encLeft_val = encLeft_val | ( ((PIND & 0b100) >> 1) | ((PIND & 0b10000) >> 4) );

  /* The previous 2-bit code and the current 2-bit code together create
   * a unique ID able to identify which direction (CW or CCW) the motor
   * attached to the encoder has moved.
   */ 
  encLeftCount = encLeftCount + encoder_lookup_table[encLeft_val & 0b1111];
}
void encoderRight_isr() {
  static uint8_t encRight_val = 0;

  encRight_val = encRight_val << 2; // Store the previous 2-bit code

  encRight_val = encRight_val | ( ((PIND & 0b1000) >> 2) | ((PIND & 0b10000000) >> 7) );

  encRightCount = encRightCount + encoder_lookup_table[encRight_val & 0b1111];
}


/************************
 * ROS Global Variables *
 ************************/

ros::NodeHandle nh;

// Declare topic names in PROGMEM
const char encoder_topic[] PROGMEM = { "/odom/encTicks" };
const char ping_time_topic[] PROGMEM = { "/ping/timeUS" };
const char ping_angle_topic[] PROGMEM = { "/ping/angleDeg" };
const char cmd_left_motors_topic[] PROGMEM = { "/cmd/left" };
const char cmd_right_motors_topic[] PROGMEM = { "/cmd/right" };

/** 
 *  Ros publishers for encoder ticks, range ping time, 
 *  and servo angle
 */
auto_rover::EncCount encMsg;
std_msgs::UInt16 pingUSMsg;
std_msgs::Int8 pingAngleDegMsg;

ros::Publisher encPub(FCAST(encoder_topic), &encMsg);
ros::Publisher pingTimePub(FCAST(ping_time_topic), &pingUSMsg);
ros::Publisher pingAngleDegPub(FCAST(ping_angle_topic), &pingAngleDegMsg);

/**
 * Callbacks for cmd_left_motors_topic and cmd_right_motors_topic topics
 * The motor power connectors were swapped on one of the motors, so that
 * both sides of the rover go forward when given a higher ON pulse width
 */
void leftMotors_cb( const std_msgs::Int8& cmd_msg){
  if (cmd_msg.data < -50 || cmd_msg.data > 50)
    nh.logerror(F("Bad cmd"));
  else
    // positive data implies go forward, which corresponds to a higher R/C pulse width
    leftMotors.write(MOTOR_CENTER + cmd_msg.data); // set servo angle, should be from 0-180
}
void rightMotors_cb( const std_msgs::Int8& cmd_msg){
  if (cmd_msg.data < -50 || cmd_msg.data > 50)
    nh.logerror(F("Bad cmd"));
  else
    // positive data implies go forward, which corresponds to a higher R/C pulse width
    rightMotors.write(MOTOR_CENTER + cmd_msg.data); // set servo angle, should be from 0-180
}

/** 
 *  Ros subscribers for cmd_left_motors_topic and cmd_right_motors_topic
 *  topics, linking to callbacks
 */
ros::Subscriber<std_msgs::Int8> leftMotorsSub(FCAST(cmd_left_motors_topic), leftMotors_cb);
ros::Subscriber<std_msgs::Int8> rightMotorsSub(FCAST(cmd_right_motors_topic), rightMotors_cb);



/**
 * This function is executed once, when the board is first
 * powered on.
 */
void setup() {
  
   /*******************************
   * Setup Arduino hardware stuff *
   ********************************/
   
  /*
   *  Attach the correct digital pins to the
   *  Servo objects. We'll use these objects
   *  to write command R/C servo pulse signals.
   */
  rightMotors.attach(rightMotorsPin); // Attach motor driver S1 to its digital pin
  leftMotors.attach(leftMotorsPin); // Attach motor driver S2 to its digital pin
  // Set min and max HIGH pulse times for the standard servo in microseconds.
  sonicServo.attach(sonicServoPin, 750, 2250); // Attach standard servo to its digital pin, datasheet specifies 0.75 - 2.25 ms high pulse
  sonicServo.write(SERVO_RIGHT); // Set its initial position all the way to the right

  /*
   * Set up hardware interrupts on the Output A pins for both
   * the encoders. Digital pins default to INPUT mode, so we 
   * don't have to set that here.
   */
  attachInterrupt(digitalPinToInterrupt(encLeftAPin), encoderLeft_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRightAPin), encoderRight_isr, CHANGE);


  /*******************
   * Setup ROS stuff *
   *******************/
  
  nh.getHardware()->setBaud(BAUD_RATE); // adjust baud rate before initializing this node
  
  // Register this node with Master
  nh.initNode();

  // Advertise sensor publishers
  nh.advertise(encPub);
  //nh.spinOnce();
  nh.advertise(pingTimePub);
  //nh.spinOnce();
  nh.advertise(pingAngleDegPub);

  // Subscribe to motor command topics
  nh.subscribe(leftMotorsSub);
  //nh.spinOnce();
  nh.subscribe(rightMotorsSub);
  
  // Wait until we are fully connected to ROS master
  while (!nh.connected() ){
    nh.spinOnce();
  }
  
  // Ok we're done and ready to start the main loop()
  nh.loginfo(F("Setup() finished."));
  nh.spinOnce();
}

/**
 * This function is executed repeatedly after setup().
 */
void loop() {
  uint8_t servoPos;
  uint16_t ping_time_uS;
  unsigned long timer;

  servoPos = SERVO_RIGHT; // initial servo position
  
  while (servoPos < SERVO_LEFT) {
    sonicServo.write(servoPos);
    
    // Listen for callbacks while we wait for the servo to reach servoPos,
    // and make sure we don't take too many pings per second.
    timer = millis();
    while (millis() - timer <  SERVO_STEP_DELAY) {
      nh.spinOnce(); // handle callbacks, flush serial buffers
    }
    
    ping_time_uS = sonar.ping(); // Send ping, get distance in uS (0 == NO_ECHO => outside set distance range)

    publishSensorMessages(servoPos, ping_time_uS);

    //nh.spinOnce(); // handle publishing callbacks immediately to flush serial buffers

    servoPos += SERVO_STEP_SZ;
  }

  servoPos = SERVO_LEFT; // in case we overshot in the previous while loop
  
  while (servoPos > SERVO_RIGHT) {
    sonicServo.write(servoPos);

    timer = millis();
    // Listen for callbacks while we wait for the servo to reach servoPos,
    // and make sure we don't take too many pings per second.
    while (millis() - timer <  SERVO_STEP_DELAY) {
      nh.spinOnce(); // handle callbacks, flush serial buffers
    }
    
    ping_time_uS = sonar.ping(); // Send ping, get distance in uS (0 == NO_ECHO => outside set distance range)

    publishSensorMessages(servoPos, ping_time_uS);

    //nh.spinOnce(); // handle publishing callbacks immediately to flush serial buffers

    // decrement by step size, but don't undershoot
    servoPos -= SERVO_STEP_SZ;
  }
  
}

/**
 * Push sensor data over serial TX. The data is encoded
 * in ASCII, so each sensor has its own unique start byte.
 * This routine pushes 26 bytes over serial per execution.
 * This routine should not be called more often than
 * (bytes per second / bytes pushed per execution) times per second.
 * The baud rate is 9600, so this routine should 
 * be called less than (960 / 26) =~ 36 times per second.
 * This routine is called roughly (1000 / 50) = 20
 * times per second. (This gives an encoder resolution
 * of 20 Hz). Thus 20 * 26 = 520 bytes per second are 
 * sent over TX.
 */
void publishSensorMessages(uint8_t servoPos, uint16_t ping_time_uS) {
  int8_t t;

  // Fill in encoder sensor message
  /*
   * Reading from multi-byte variables which are accessed within
   * and without an ISR risks data corruption, so interrupt guards
   * must be used around a read to make it atomic. Important to not
   * disable all interrupts, as serial RX interrupts occur at a high
   * rate when we set the baud rate high.
   */
  detachInterrupt(digitalPinToInterrupt(encLeftAPin)); // turn off handling of left encoder pulse edges
  encMsg.leftTicks = encLeftCount; // long is 4 bytes, so assignment translates to 4 machine instructions.
  attachInterrupt(digitalPinToInterrupt(encLeftAPin), encoderLeft_isr, CHANGE); // reattach interrupts, will execute any flagged events
  // The next statement is guaranteed to run before flagged interrupts are handled (I actually don't want this feature, but wtv, we should have time to do it)
  t = 0; // do a quick useless statement so that a flagged interrupt is handled quickly
  
  detachInterrupt(digitalPinToInterrupt(encRightAPin)); // turn off handling of right encoder pulse edges
  // worst case scenario, encoder interrupt flag set right after detachInterrupt()
  encMsg.rightTicks = encRightCount; // 4 machine instructions
  attachInterrupt(digitalPinToInterrupt(encRightAPin), encoderRight_isr, CHANGE); // reattach interrupts, will execute any flagged events
  t = 0; // do a quick useless statement so that flagged interrupts are handled quickly

  // negate left count to make the two ticks both positive as the rover moves forward
  encMsg.leftTicks = -1 * encMsg.leftTicks;

  // Fill in ultrasonic sensor messages
  pingUSMsg.data = ping_time_uS;
  // Transmit angle in degrees from x-axis of base_link, with CCW positive.
  // Follows REP-103: http://www.ros.org/reps/rep-0103.html#rotation-representation
  pingAngleDegMsg.data = (int8_t) (servoPos - SERVO_CENTER); 
  
  // Publish sensor messages
  encPub.publish(&encMsg);
  pingTimePub.publish(&pingUSMsg);
  pingAngleDegPub.publish(&pingAngleDegMsg);
}



