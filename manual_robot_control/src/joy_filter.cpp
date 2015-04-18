#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Header.h>

ros::Subscriber sub_joy;
ros::Publisher pub;

// square function
inline double pow2(double a) {return a * a;}

// controller deadzones
// TODO: makes these use parameters
double x_deadzone = 0.06;
double z_deadzone = 0.12;
double change_tol = 0.05;
unsigned int max_stick_rate = 10;
unsigned int min_rate = 2;

// header stuff
unsigned int seq = 0;

// store previous message
sensor_msgs::Joy prev = sensor_msgs::Joy();

void callback_joy(const sensor_msgs::Joy::ConstPtr & tmpjoy)
{
  sensor_msgs::Joy joy = *tmpjoy;

  // check deadzone
  if (joy.axes[0] > -z_deadzone && joy.axes[0] < z_deadzone) joy.axes[0] = 0;
  if (joy.axes[1] > -x_deadzone && joy.axes[1] < x_deadzone) joy.axes[1] = 0;
   
  bool changed = false;

  // check if buttons changed
  for (int i = 0; i < 8; i++)
  {
    if (joy.buttons[i] != prev.buttons[i])
    {
      changed = true;
      break;
    }
  }
  
  // check if axes changed more than tolerance and elapsed time
  unsigned int dt = (joy.header.stamp - prev.header.stamp).toNSec() / 1000000;
  if (dt > 1000 / min_rate)
    changed = true;

  if (dt > 1000 / max_stick_rate)
  {
    for (int i = 0; i < 2; i++)
    {
      if (pow2(joy.axes[i] - prev.axes[i]) > pow2(change_tol))
      {
        changed = true;
        break;
      }
    }
  }

  if (changed)
  {
    for (int i = 0; i < 8; i++) prev.buttons[i] = joy.buttons[i];
    for (int i = 0; i < 2; i++) prev.axes[i] = joy.axes[i];
    prev.header.stamp.sec = joy.header.stamp.sec;
    prev.header.stamp.nsec = joy.header.stamp.nsec;

    joy.header.seq = seq++;
    joy.header.frame_id = "";
    joy.header.stamp = ros::Time::now();
    pub.publish(joy);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_filter");
  ros::NodeHandle n;
  pub = n.advertise<sensor_msgs::Joy>("/joy_filtered", 1);
  
  sub_joy = n.subscribe<sensor_msgs::Joy>("/joy", 1, callback_joy);

  prev.axes.resize(8);
  prev.buttons.resize(11);
  for (int i = 0; i < 8; i++) prev.buttons[i] = 0;
  for (int i = 0; i < 2; i++) prev.axes[i] = 0.;

  ros::spin();

  return 0;
}
