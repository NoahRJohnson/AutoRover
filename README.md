# AutoRover
![Constructed Rover](https://github.com/NoahRJohnson/Thesis/blob/master/figures/roverFinished.jpg)
My New College of Florida undergraduate honors thesis project to try to create an autonomous ground vehicle.

The rover base consists of a Lynxmotion rover, an Arduino, and a Sabertooth motor driver. Sensors available include two quadrature rotary encoders, a mobile phone, and an ultrasonic distance sensor.

The Arduino acts as a low-level robotic controller, publishing wheel encoder and range data, and accepting motor velocity commands. An Android app publishes IMU and GPS data from the mobile phone, and the laptop fuses these readings into a state estimation using the robot_localization package.

See my [Thesis](https://github.com/NoahRJohnson/Thesis/blob/master/main.pdf) for a detailed look into the project. 

To use the auto_rover ROS package, simply copy auto_rover into the src folder of your catkin workspace, and then run 'catkin_make' and 'catkin_make install' from your workspace folder.

To use the Arduino sketch, open rover_sketch.ino in the Arduino IDE, connect your Arduino board with a USB printer cable, and upload the program. To build the RosSensors app onto your personal mobile phone, use Android Studio. Your phone will need to have a magnetometer, gyroscope, accelerometer, and GPS receiver.

