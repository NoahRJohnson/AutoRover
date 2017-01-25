#include <Servo.h>

// Make sure that Arduino GND is connected to motor driver's GND

// Define Arduino digital pin connections 
#define enc1APin 2 // Hardware interrupt pin
#define enc1BPin 4
#define enc2APin 3 // Hardware interrupt pin
#define enc2BPin 7
#define throttlePin 5
#define turnPin 6
#define sonicServoPin 9
#define ultrasonicPin 11


Servo throttle, turn; // Sabertooth R/C control. 0 deg full forward, 90 stopped, 180 deg full reverse
Servo sonicServo; // standard Parallax servo attached to ultrasonic sensor

int servoPos = 90;

void setup()
{
  Serial.begin(115200);

  // Sabertooth accepts servo pulses from 1000 us to 2000 us.
  //throttle.attach(throttlePin); //, 1000, 2000
  //turn.attach(turnPin); //, 1000, 2000
  
  sonicServo.attach(sonicServoPin);
  
  // Tell motors not to move.
  //throttle.write(92);
  //turn.write(92);

  //sonicServo.write(144); // Set servo to center
  
  //delay(5000);
}

void loop()
{
  sonicServo.write(180);   // Rotate servo counter clockwise
  delay(2000);          // Wait 2 seconds
  sonicServo.write(0);     // Rotate servo clockwise
  delay(2000);
  sonicServo.write(90);    // Rotate servo to center
  delay(2000); 
  
  /*while (servoPos < 180) {
    // PUT PING))) CODE HERE
    sonicServo.write(servoPos);
    delay(15); // wait 15 ms for servo to reach pos 
    servoPos++;
  }

  while (servoPos > 0) {
    //PUT PING))) CODE HERE
    sonicServo.write(servoPos);
    delay(15); // wait 15 ms for servo to reach pos
    servoPos--;
  }*/

}
