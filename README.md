# AutoRover
My New College of Florida undergraduate honors thesis project.

Using a laptop, an Arduino, a Lynxmotion rover, a mobile phone and an ultrasonic distance sensor to create an autonomous rover.

All components communicate with ROS. The Arduino acts as a low-level robotic controller, publishing wheel encoder and range data, and accepting motor velocity commands. An Android app publishes IMU and GPS data from the mobile phone, and the laptop fuses these readings into a state estimation using the robot_localization package.

See my [Thesis](https://github.com/NoahRJohnson/Thesis/blob/master/main.pdf) for a detailed look into the project. 

![Constructed Rover](https://github.com/NoahRJohnson/Thesis/blob/master/figures/roverFinished.jpg)
