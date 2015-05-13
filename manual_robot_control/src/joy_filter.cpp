#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Header.h>
#include <string>

ros::Subscriber sub_joy;
ros::Publisher pub;

// square function
inline double pow2(double a) {return a * a;}

double x_deadzone;
double z_deadzone;
double jitter_tol;
double max_stick_rate;
double min_rate;

// header stuff
unsigned int seq = 0;

// store previous message
sensor_msgs::Joy prev = sensor_msgs::Joy();

// the topic to publish
std::string topic;

void callback_joy(const sensor_msgs::Joy::ConstPtr & tmpjoy)
{
  sensor_msgs::Joy joy = *tmpjoy;

  bool changed = false;

  // check deadzone
  if (joy.axes[0] > -z_deadzone && joy.axes[0] < z_deadzone) joy.axes[0] = 0;
  if (joy.axes[1] > -x_deadzone && joy.axes[1] < x_deadzone) joy.axes[1] = 0;
   
  // check if buttons changed
  for (int i = 0; i < 8; i++)
  {
    if (joy.buttons[i] != prev.buttons[i])
    {
      changed = true;
      break;
    }
  }

  if ((joy.axes[0] == 0 && joy.axes[1] == 0) &&
      (prev.axes[0] != 0 || prev.axes[0] != 0))
  {
    changed = true;
  }
  
  // check if axes changed more than tolerance and elapsed time
  unsigned int dt = (joy.header.stamp - prev.header.stamp).toNSec() / 1000000;
  if (dt > 1000 / min_rate)
    changed = true;

  if (dt > 1000 / max_stick_rate)
  {
    for (int i = 0; i < 2; i++)
    {
      if (pow2(joy.axes[i] - prev.axes[i]) > pow2(jitter_tol))
      {
        changed = true;
        break;
      }
    }
  }

  if ((joy.axes[0] == 0 && joy.axes[1] == 0) && 
      (prev.axes[0] != 0 || prev.axes[1] != 0))
    changed = true;

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
  ros::NodeHandle n_("~");

  // set the parameters
  n_.param<double>("threshold/x", x_deadzone, 0.10);
  n_.param<double>("threshold/z", z_deadzone, 0.12);
  n_.param<double>("threshold/jitter", jitter_tol, 0.05);
  n_.param<double>("max_rate", max_stick_rate, 10);
  n_.param<double>("min_rate", min_rate, 2);
  n_.param<std::string>("topic", topic, "/IRIS/joy_filtered");

  pub = n.advertise<sensor_msgs::Joy>(topic, 1);
  
  sub_joy = n.subscribe<sensor_msgs::Joy>("/joy", 1, callback_joy);

  prev.axes.resize(8);
  prev.buttons.resize(11);
  for (int i = 0; i < 8; i++) prev.buttons[i] = 0;
  for (int i = 0; i < 2; i++) prev.axes[i] = 0.;

  ros::spin();

  return 0;
}
