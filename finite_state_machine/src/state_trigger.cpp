#include <ros/ros.h>
#include <std_msgs/Bool.h>

// just publish a message with specified frequency (Rate)
int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_trigger");
  ros::NodeHandle n;
  unsigned int frequency = 2;
  ros::Rate loop_rate(frequency);
  ros::Publisher pub = n.advertise<std_msgs::Bool>("/IRIS/trigger", 1);

  while (ros::ok())
  {
    std_msgs::Bool msg;
    pub.publish(msg);

    loop_rate.sleep();
  }
 
  return 0;
}
