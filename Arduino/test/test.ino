#include <Servo.h>

// Make sure that Arduino GND is connected to motor driver's GND

// Define Arduino digital pin connections 
#define enc1APin 2 // Hardware interrupt pin
#define enc1BPin 4
#define enc2APin 3 // Hardware interrupt pin
#define enc2BPin 7
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

char storedCommand = 0; // Global variable to indicate whether a command has been read (non-zero), and store what that command was

void setup()
{
  Serial.begin(115200);

  // Control rover through sabertooth r/c mode turn and throttle sticks
  throttle.attach(throttlePin);
  turn.attach(turnPin);
  
  sonicServo.attach(sonicServoPin);
  
  // Tell motors not to move.
  //throttle.write(130);
  //turn.write(90);

  // Set servo to starting position
  servoPos = SERVO_RIGHT;
  sonicServo.write(servoPos);
  
  //delay(5000);
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
      Serial.print("ERROR: Unrecognized command symbol: "); Serial.println(storedCommand);
    }
  }
}

void loop()
{
  
  while (servoPos < SERVO_LEFT) {
    sonicServo.write(servoPos);
    if (Serial.available() > 1) {
      processSerialInput();
    } else {
      delay(15); // wait 15 ms for servo to reach pos 
    }
    Serial.print("Ping: ");
    Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.println("cm");
    servoPos = servoPos + SERVO_STEP_SZ;
  }

  while (servoPos > SERVO_RIGHT) {
    sonicServo.write(servoPos);
    if (Serial.available() > 1) {
      processSerialInput();
    } else {
      delay(15); // wait 15 ms for servo to reach pos 
    }
    Serial.print("Ping: ");
    Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.println("cm");
    servoPos = servoPos - SERVO_STEP_SZ;
  }

}
