/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <auto_rover/EncCount.h>

ros::NodeHandle  nh;

//const char str_topic[] PROGMEM = { "chat" };
const char encoder_topic[] PROGMEM = { "/odom/encTicks" };

//std_msgs::String str_msg;
//ros::Publisher chatter(FCAST(str_topic), &str_msg);

//char hello[13] = "hello world!";

auto_rover::EncCount encMsg;

ros::Publisher encPub(FCAST(encoder_topic), &encMsg);

void setup()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();

  /*
  // Set up encoder ticks message to use one-dimensional array of two elements
  encMsg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension)); // 1 dim array
  encMsg.layout.dim_length = 1; // http://wiki.ros.org/rosserial/Overview/Limitations#Arrays
  encMsg.layout.dim[0].label = encMsg_dim0_label; // http://wiki.ros.org/rosserial/Overview/Limitations#Strings
  encMsg.layout.dim[0].size = 2; // 2 elements in array
  encMsg.layout.dim[0].stride = 2;
  encMsg.layout.data_offset = 0;
  encMsg.data = (long *)malloc(sizeof(long) * 2); // 2 elements, 4 bytes (32 bits) each
  encMsg.data_length = 2; // http://wiki.ros.org/rosserial/Overview/Limitations#Arrays
  */
  //nh.advertise(chatter);
  nh.advertise(encPub);
  nh.loginfo(F("Setup() finished."));
  nh.spinOnce();
}

void loop()
{
  long a = 1;
  long b = 2;
  
  encMsg.leftTicks = a; // long is 4 bytes, so assignment translates to 4 machine instructions.
  encMsg.rightTicks = b; // also 4 machine instructions
  encMsg.stamp = nh.now();
  
  encPub.publish(&encMsg);
  nh.spinOnce();
  delay(1000);
}
