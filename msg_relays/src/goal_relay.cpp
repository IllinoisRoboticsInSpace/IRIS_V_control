#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <IRIS_msgs/RobotCommandStamped.h>
#include <string>

ros::Subscriber sub_;
ros::Publisher pub_;
std::string subtopic;
geometry_msgs::PoseStamped goal;

void callback(const IRIS_msgs::RobotCommandStamped::ConstPtr & command)
{
  goal.pose = command->command.goal;
  goal.header = command->header;
  pub_.publish(goal);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_relay");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  nh_.param<std::string>("command_topic", subtopic, "/IRIS/status");

  pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  sub_ = nh.subscribe<IRIS_msgs::RobotCommandStamped>(subtopic, 1, callback);

  ros::spin();

  return 0;
}
