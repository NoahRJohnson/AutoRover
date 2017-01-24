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
  throttle.attach(throttlePin, 1000, 2000);
  turn.attach(turnPin, 1000, 2000);
  
  sonicServo.attach(sonicServoPin);
  
  // Tell motors not to move.
  hrottle.write(130);
  //turn.write(90);

  sonicServo.write(servoPos); // Set servo to center
  
  //delay(5000);
}

void serialEvent() { // Called when data is sent by laptop over serial port

  char c, d1, d2, d3;
  int deg;
    
  while (Serial.available()) {
    
    c = (char) Serial.read();

    if (c == 'P') { // Power
      d1 = Serial.read();
      d2 = Serial.read();
      d3 = Serial.read();

      deg = (d1 - '0') * 100 + (d2 - '0') * 10 + (d3 - '0');
      // TODO: Log deg?
      throttle.write(deg);
    } else if (c == 'T') { // Turn
      d1 = Serial.read();
      d2 = Serial.read();
      d3 = Serial.read();

      deg = (d1 - '0') * 100 + (d2 - '0') * 10 + (d3 - '0');
      // TODO: Log deg?
      turn.write(deg);
    } else { // unrecognized command
      Serial.print("ERROR: Unrecognized command symbol: "); Serial.println(c);
    }

  }
}

void loop()
{
  
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
