# AutoRover
![Constructed Rover](https://github.com/NoahRJohnson/Thesis/blob/master/figures/roverFinished.jpg)
My New College of Florida undergraduate honors thesis project to try to create an autonomous ground vehicle.

The rover base consists of a Lynxmotion rover, an Arduino, and a Sabertooth motor driver. Sensors available include two quadrature rotary encoders, a mobile phone, and an ultrasonic distance sensor.

The Arduino acts as a low-level robotic controller, publishing wheel encoder and range data, and accepting motor velocity commands. An Android app publishes IMU and GPS data from the mobile phone, and the laptop fuses these readings into a state estimation using the robot_localization package.

See my [Thesis](https://github.com/NoahRJohnson/Thesis/blob/master/main.pdf) for a detailed look into the project. 

To use the auto_rover ROS package, simply copy auto_rover into the src folder of your catkin workspace, and then run 'catkin_make' and 'catkin_make install' from your workspace folder.

To use the Arduino sketch, download the modified rosserial package [here](https://github.com/NoahRJohnson/rosserial). Then use 'rosrun rosserial_arduino make_libraries.py /path/to/sketchbook/libraries/' to construct the ros_lib Arduino library. This modified version of rosserial supports the use of PROG_MEM string constants, which save dynamic memory on the Arduino. Also install the [NewPing]() library into your sketchbook/libraries folder. Now you should be good to go, so open rover_sketch.ino in the Arduino IDE, connect your Arduino board with a USB printer cable, and upload the program. 

Right now to build the RosSensors app onto your personal mobile phone, you have to build from source using Android Studio. Your phone will need to have a magnetometer, gyroscope, accelerometer, and GPS receiver. Connect your phone to your machine via a micro USB cable, and open the RosSensors project in Android Studio. You may have to fiddle with udev rules to make your machine recognize your phone, see [here](https://developer.android.com/studio/run/device.html) for instructions on that. Once your phone is recognized, select Build -> Run, and after a delay the app should open up on your phone. You should now find it under your Apps even after disconnecting the phone from your computer. To actually use the app, make sure a ROS Master instace is running on your machine, that your phone and machine are connected via USB, that your udev rules allow your phone to connect, that USB debugging mode is enabled, and that USB tethering is on. Then use ifconfig to find your phone's IP address, and enter that into the ROS Master chooser window and click Connect. If you're successful the ap should say 'Connected', and you can view the data stream with 'rostopic list' 'rostopic echo ...'
