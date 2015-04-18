#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <IRIS_msgs/RobotCommandStamped.h>
#include <IRIS_msgs/RobotStatusStamped.h>
#include <serial/serial.h>

serial::Serial ser;
std::string send = "abc";

void callback_heartbeat(const std_msgs::Bool::ConstPtr & heartbeat)
{
  ROS_INFO("LINE 12");
  ROS_INFO_STREAM("Writing to serial port: " << send);
  ROS_INFO("LINE 14");
  ser.write(send);
  ROS_INFO("LINE 16");
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "serial_communication");
  ROS_INFO("LINE 22");
  ros::NodeHandle n;
  ROS_INFO("LINE 24");

//ros::Subscriber sub_cmd = n.subscribe<IRIS_msgs::RobotCommandStamped>("IRIS/command", 1, callback_cmd);
  ros::Subscriber sub_heartbeat = n.subscribe<std_msgs::Bool>("/IRIS/heartbeat", 1, 
                                                              callback_heartbeat);
  ROS_INFO("LINE 29");

  ros::spin();
  ROS_INFO("LINE 32");

  return 0;
}
