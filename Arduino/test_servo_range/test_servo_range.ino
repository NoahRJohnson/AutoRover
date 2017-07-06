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

void setup() {
  // Set min and max HIGH pulse times for the standard servo in microseconds.
  sonicServo.attach(sonicServoPin, 750, 2250); // Attach standard servo to its digital pin, datasheet specifies 0.75 - 2.25 ms high pulse
  sonicServo.write(SERVO_RIGHT); // Set its initial position all the way to the right
}
void loop() {
  uint8_t servoPos;
  uint16_t ping_time_uS;
  unsigned long timer;

  servoPos = SERVO_RIGHT; // initial servo position
  
  while (servoPos < SERVO_LEFT) {
    sonicServo.write(servoPos);
    
    timer = millis();
    while (millis() - timer <  SERVO_STEP_DELAY) {
      ; // do nothing
    }

    servoPos += SERVO_STEP_SZ;
  }

  servoPos = SERVO_LEFT; // in case we overshot in the previous while loop
  
  while (servoPos > SERVO_RIGHT) {
    sonicServo.write(servoPos);

    timer = millis();
    while (millis() - timer <  SERVO_STEP_DELAY) {
      ; // do nothing
    }
    servoPos -= SERVO_STEP_SZ;
  }
  
}


