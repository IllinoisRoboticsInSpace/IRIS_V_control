#include <ros/ros.h>
#include <IRIS_msgs/RobotStatus.h>
#include <nav_msgs/Odometry.h>
#include <string>

ros::Subscriber sub_;
ros::Publisher pub_;
std::string subtopic;
nav_msgs::Odometry odom;

void callback(const IRIS_msgs::RobotStatus::ConstPtr & status)
{
  odom = status->odom;
  pub_.publish(odom);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_relay");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  nh_.param<std::string>("status_topic", subtopic, "/IRIS/status");

  pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  sub_ = nh.subscribe<IRIS_msgs::RobotStatus>(subtopic, 1, callback);

  ros::spin();

  return 0;
}

