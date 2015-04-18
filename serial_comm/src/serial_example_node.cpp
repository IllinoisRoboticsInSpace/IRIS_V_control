/***
* This example expects the serial port has a loopback on it.
*
* Alternatively, you could use an Arduino:
*
* <pre>
* void setup() {
  * Serial.begin(<insert your baudrate here>);
  * }
  *
  * void loop() {
    * if (Serial.available()) {
      * Serial.write(Serial.read());
      * }
      * }
      * </pre>
      */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
serial::Serial ser;
std::string send = "a";

int main (int argc, char** argv){
  ros::init(argc, argv, "serial_example_node");
  ros::NodeHandle nh;
  try
  {
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(9600);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }
  if(ser.isOpen()){
    ROS_INFO_STREAM("Serial Port initialized");
  }else{
    return -1;
  }

  ros::Rate loop_rate(2);
  while(ros::ok()){
    ROS_INFO("LINE 49");
    ros::spinOnce();
    ROS_INFO("LINE 51");
    ROS_INFO_STREAM("Writing to serial port: " << send);
    ROS_INFO("LINE 53");
    ser.write(send);
    ROS_INFO("LINE 55");
    send[0]++;
    ROS_INFO("LINE 57");

//  while(!ser.available()) {}

//  if(ser.available()){
//    ROS_INFO_STREAM("Reading from serial port");
//    std::string result;
//    ROS_INFO("LINE 62");
//    result = ser.read(ser.available());
//    ROS_INFO("LINE 64");
//    ROS_INFO_STREAM("Read: " << result);
//    ROS_INFO("LINE 66");
//  }
//  ROS_INFO("LINE 68");
//  loop_rate.sleep();
    ROS_INFO("LINE 70");
  }
}

