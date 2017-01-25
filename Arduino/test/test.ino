#include <Servo.h>
#include <NewPing.h>

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



#define TRIGGER_PIN  11  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


Servo throttle, turn; // Sabertooth R/C control. 0 deg full forward, 90 stopped, 180 deg full reverse
Servo sonicServo; // standard Parallax servo attached to ultrasonic sensor

int servoPos = 90;

void setup()
{
  Serial.begin(115200);
  Serial.println("Begin");

  // Sabertooth accepts servo pulses from 1000 us to 2000 us.
  //throttle.attach(throttlePin); //, 1000, 2000
  //turn.attach(turnPin); //, 1000, 2000
  
  sonicServo.attach(sonicServoPin);
  
  // Tell motors not to move.
  //throttle.write(92);
  //turn.write(92);

  sonicServo.write(66); // Set servo to center
  
  //delay(5000);
}

void loop()
{

    delay(333); // wait 15 ms for servo to reach pos 
    Serial.print("Ping: ");
    Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.println("cm");
    /*
  while (servoPos < 180) {
    // PUT PING))) CODE HERE
    sonicServo.write(servoPos);
    delay(15); // wait 15 ms for servo to reach pos 
    Serial.print("Ping: ");
    Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.println("cm");
    servoPos++;
  }

  while (servoPos > 0) {
    //PUT PING))) CODE HERE
    sonicServo.write(servoPos);
    delay(15); // wait 15 ms for servo to reach pos
    Serial.print("Ping: ");
    Serial.println(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.println("cm");
    servoPos--;
  }*/

}
