#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>

int frequency;
std::string topic = "";

// just publish a message with specified frequency (Rate)
int main(int argc, char **argv)
{
  ros::init(argc, argv, "trigger");
  ros::NodeHandle n;
  ros::NodeHandle n_("~");

  // set parameters
  n_.param<std::string>("topic", topic,"trigger");
  n_.param<int>("frequency", frequency, 1);

  ros::Rate loop_rate(frequency);
  ros::Publisher pub = n.advertise<std_msgs::Bool>(topic, 1);

  while (ros::ok())
  {
    std_msgs::Bool msg;
    pub.publish(msg);

    loop_rate.sleep();
  }
 
  return 0;
}
