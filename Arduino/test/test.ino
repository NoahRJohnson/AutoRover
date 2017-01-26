#include <Servo.h>
#include <NewPing.h>

// Make sure that Arduino GND is connected to motor driver's GND

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

Servo throttle, turn; // Sabertooth R/C control. 0 deg full forward, 90 stopped, 180 deg full reverse
Servo sonicServo; // standard Parallax servo attached to ultrasonic sensor. xx left, 66 center, xx 

#define SERVO_LEFT 130
#define SERVO_CENTER 66
#define SERVO_RIGHT 0
#define SERVO_STEP_SZ 5
int servoPos;

/*
 * Signed longs are 4 bytes long (about -2 billion to 2 billion).
 */
volatile long enc1_count = 0L;
volatile long enc2_count = 0L;

void setup()
{
  Serial.begin(115200);

  // Control rover through sabertooth r/c mode turn and throttle sticks
  //throttle.attach(throttlePin);
  //turn.attach(turnPin);
  
  //sonicServo.attach(sonicServoPin);
  
  // Tell motors not to move.
  //throttle.write(130);
  //turn.write(90);

  // Set servo to starting position
  servoPos = SERVO_RIGHT;
  //sonicServo.write(servoPos);

  
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
  static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
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
  enc1_count = enc1_count + lookup_table[enc_val & 0b1111];
}

/*
 * Interrupt Service Routine
 * http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
 * 
 *  ISRs must be as fast as possible, so rather than clear coding practice (like using
 * (1 << enc2APin), I will minimize operations by using constants where I can (0b1000).
 */
void encoder2_isr() {
  static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
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
  enc2_count = enc2_count + lookup_table[enc_val & 0b1111]; 
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
      Serial.print("ERROR: Unrecognized command symbol: "); Serial.println(command);
    }
  }
}

void loop()
{
  
  Serial.print("Enc1: "); Serial.println(enc1_count);
  Serial.print("Enc2: "); Serial.println(enc2_count);
  delay(50);

}
