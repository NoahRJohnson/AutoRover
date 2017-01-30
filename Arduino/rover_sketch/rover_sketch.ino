#include <Servo.h>
#include <NewPing.h>

// Make sure that Arduino GND is connected to motor driver's GND

/**
 * Number of symbols that will be transferred over serial port
 * per second. Arduino serial defaults to 8N1 which uses 10 symbols
 * per byte of data (1 start symbol, 8 data bits, 1 stop symbol).
 */
#define BAUD_RATE 9600 // 960 bytes per second, roughly 1 ms between bytes

// Define Arduino digital pin connections 
#define enc1APin 2 // Hardware interrupt pin, channel A output
#define enc1BPin 4 // Channel B output
#define enc2APin 3 // Hardware interrupt pin, channel A output
#define enc2BPin 7 // Channel B output
#define throttlePin 5 // S1 on Sabertooth
#define turnPin 6 // S2 on Sabertooth
#define sonicServoPin 9 // standard Parallax servo attached to ultrasonic sensor
#define TRIGGER_PIN  11  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Setup NewPing to use the appropriate pins and settings

Servo throttle, turn; // Sabertooth R/C control. 44 deg full forward, 94 stopped, 144 deg full reverse
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
#define SERVO_STEP_DELAY 50 // How many ms to wait in-between servo steps, and thus between pings
#define SERVO_DEFAULT_DEG 93 // Which angular position the servo defaults to when attached but given no directions.

/*
 * Signed longs are 4 bytes long (about -2 billion to 2 billion).
 */
volatile long enc1_count = 0L;
volatile long enc2_count = 0L;

static int8_t encoder_lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};

void setup() {
  Serial.begin(BAUD_RATE);

  // Control rover through sabertooth r/c mode turn and throttle sticks
  throttle.attach(throttlePin);
  turn.attach(turnPin);

  // Control standard servo
  sonicServo.attach(sonicServoPin);
  sonicServo.write(SERVO_RIGHT);

  // Set up hardware interrupts on the Output A pins for both
  // the encoders. The Uno only has 2 hardware interrupt pins.
  // Digital pins default to INPUT mode, so we don't have to set
  // that here.
  attachInterrupt(digitalPinToInterrupt(enc1APin), encoder1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2APin), encoder2_isr, CHANGE);
  
  //delay(5000);
}

/*
 * Interrupt Service Routine
 * http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
 * 
 * ISRs must be as fast as possible, so rather than clear coding practice (like using
 * (1 << enc1APin), I will conserve operations by using constants where I can (0b100).
 */
void encoder1_isr() {
  static uint8_t enc_val = 0;

  enc_val = enc_val << 2; // Store the previous 2-bit code

  /* 
   *  PIND returns input readings from pins 0-7 as a byte.
   *  https://www.arduino.cc/en/Reference/PortManipulation
   *  The below code uses bit shifting and masking to put
   *  the values of digital input pins 4 (output B) and 
   *  2 (output A) into the least significant bit and 
   *  second-least significant bit of the enc_val 
   *  variable, respectively.
   */
  enc_val = enc_val | ( ((PIND & 0b100) >> 1) | ((PIND & 0b10000) >> 4) );

  /* The previous 2-bit code and the current 2-bit code together create
   * a unique ID able to identify which direction (CW or CCW) the motor
   * attached to the encoder has moved.
   */ 
  enc1_count = enc1_count + encoder_lookup_table[enc_val & 0b1111];
}

/*
 * Interrupt Service Routine
 * http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
 * 
 *  ISRs must be as fast as possible, so rather than clear coding practice (like using
 * (1 << enc2APin), I will minimize operations by using constants where I can (0b1000).
 */
void encoder2_isr() {
  static uint8_t enc_val = 0;

  enc_val = enc_val << 2; // Store the previous 2-bit code

  /* 
   *  PIND returns input readings from pins 0-7 as a byte.
   *  https://www.arduino.cc/en/Reference/PortManipulation
   *  The below code uses bit shifting and masking to put
   *  the values of digital input pins 7 (output B) and 
   *  3 (output A) into the least significant bit and 
   *  second-least significant bit of the enc_val 
   *  variable, respectively.
   */
  enc_val = enc_val | ( ((PIND & 0b1000) >> 2) | ((PIND & 0b10000000) >> 7) );

  /* The previous 2-bit code and the current 2-bit code together create
   * a unique ID able to identify which direction (CW or CCW) the motor
   * attached to the encoder has moved.
   */ 
  enc2_count = enc2_count + encoder_lookup_table[enc_val & 0b1111];
}

/*
  This routine is run between each movement of the ultrasonic
  servo. Commands from the processing unit are expected to be
  two bytes, with the first byte indicating whether the throttle
  or turn sticks are to be modified, and the second byte indicating
  the value to change to. Multiple commands may be available from
  the Arduino's receiving buffer, which stores up to 64 bytes (or
  32 commands).
 */
void processSerialInput() { 
  byte b;
  char command;

  while (Serial.available() > 1) {
    command = (char) Serial.read();
    b = Serial.read();
    
    if (command == 'P') { // Power (throttle)
      throttle.write(b);
    } else if (command == 'T') { // Turn
      turn.write(b);
    } else { // unrecognized command
      Serial.print("E: cmd "); Serial.println(command);
    }
  }
}

/**
 * This function loops continuously while the Arduino runs.
 */
void loop() {
  uint8_t servoPos, ping_time_uS;

  servoPos = SERVO_RIGHT; // initial servo position
  
  while (servoPos < SERVO_LEFT) {
    sonicServo.write(servoPos);

    if (Serial.available() > 1) {
      processSerialInput();
    } else {
      delay(SERVO_STEP_DELAY); // wait for servo to reach pos, and make sure we don't take too many pings per second
    }
    
    ping_time_uS = sonar.ping(); // Send ping, get distance in uS (0 == NO_ECHO => outside set distance range)

    pushSensorUpdate(servoPos, ping_time_uS);

    // increment by step size, but don't overshoot
    servoPos += (servoPos + SERVO_STEP_SZ <= SERVO_LEFT) ? SERVO_STEP_SZ : (SERVO_LEFT - servoPos); 
  }

  while (servoPos > SERVO_RIGHT) {
    sonicServo.write(servoPos);
    
    if (Serial.available() > 1) {
      processSerialInput();
    } else {
      delay(SERVO_STEP_DELAY); // wait for servo to reach pos, and make sure we don't take too many pings per second
    }
    
    ping_time_uS = sonar.ping(); // Send ping, get distance in uS (0 == NO_ECHO => outside set distance range)

    pushSensorUpdate(servoPos, ping_time_uS);

    // decrement by step size, but don't undershoot
    servoPos -= (servoPos - SERVO_STEP_SZ >= SERVO_RIGHT) ? SERVO_STEP_SZ : (servoPos - SERVO_RIGHT);
  }
}

/**
 * Push sensor data over serial TX.
 * This routine currently pushes 14 bytes per execution.
 * This routine should not be called more often than
 * (baud rate / 14) times per second.
 * The baud rate is currently 9600, so this routine should 
 * be called less than 680 times per second.
 */
void pushSensorUpdate(int8_t servoPos, uint8_t ping_uS) {

  long enc1_cnt_temp, enc2_cnt_temp;

  /*
   * Reading from multi-byte variables which are accessed within
   * and without an ISR risks data corruption, so interrupt guards
   * must be used around a read to make it atomic.
   */
  noInterrupts(); // turn off all interrupts
  // worst case scenario, two encoder interrupt flags and serial RX flag are set right after noInterrupts();
  enc1_cnt_temp = enc1_count; // long is 4 bytes, so assignment translates to 4 machine instructions.
  enc2_cnt_temp = enc2_count; // also 4 machine instructions
  interrupts(); // turn all interrupts back on, run next statement, then handle any that were flagged in-between guards
  // The next statement is guaranteed to run before flagged interrupts are handled (I actually don't want this feature, but wtv, we should have time to do it)

  /* 
   *  Send sensor data to processing unit over serial port, with descriptive start bytes.
   *  L == Left motor's quadrature encoder's position value
   *  R == Right motor's quadrature encoder's position value
   *  S == Angular degree of servo
   *  P == Ultrasonic ping distance measured at the given servo angle
   */
  Serial.print("L"); 
  Serial.write(enc1_cnt_temp);
  Serial.print("R");
  Serial.write(enc2_cnt_temp);
   
  Serial.print("S");
  Serial.write(servoPos);
  Serial.print("P");
  Serial.write(ping_uS);
}

