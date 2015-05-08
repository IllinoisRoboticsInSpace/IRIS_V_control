// standard library includes
#include <cmath>
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

// ROS msg includes
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <IRIS_msgs/RobotCommand.h>
#include <IRIS_msgs/RobotCommandStamped.h>
#include <IRIS_msgs/RobotStatus.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>

#define msec2nsec 1000000

// define pi for math use
const double pi = 4 * atan(1.);

// list of states for machine
enum state {wait_to_start, localize, move_to_mine, mine, move_to_dump,
                dump, manual};
enum actuator_status {rectracting, down, extending, up};
enum actuator_command : bool {DOWN = false, UP = true};
enum paddle_status : bool {OFF = false, ON = true};

/** 
 * class to maintain shared and persistent data whie subscribing to
 * multiple topics and publishing the RobotCommand
 **/
class StateMachine
{

public:
  /**
   * StateMachine constructor
   * Subscribes to Joy command for mission control & RobotStatus for autonomy.
   **/
  StateMachine()
  {
    nh = ros::NodeHandle();
    nh_ = ros::NodeHandle("~");

    // set parametrized data
    ROS_INFO("Setting parameters (1)");
    nh_.param<std::string>("topic/command", command_topic, "/IRIS/command");
    nh_.param<std::string>("topic/status", status_topic, "/IRIS/status");
    nh_.param<std::string>("topic/joy", joy_topic, "/IRIS/joy_filtered");
    nh_.param<std::string>("topic/beat", heartbeat_topic, "/IRIS/heartbeat");
    nh_.param<std::string>("topic/trigger", trigger_topic, "/IRIS/trigger");
    nh_.param<std::string>("frame_id", frame_id, "0");
    nh_.param<int>("timeout", timeout, 1000);
    ROS_INFO("Set parameters (1)");
    ROS_INFO((command_topic + "\n" + status_topic + "\n" + joy_topic + "\n" +
             heartbeat_topic + "\n" + trigger_topic + "\n" + frame_id).c_str());
    ROS_INFO("%d", timeout);

    // topic to publish (command for the robot)
    pub_ = nh.advertise<IRIS_msgs::RobotCommandStamped>(command_topic, 1);

    // topics to subscribe
    sub_joy = nh.subscribe(joy_topic, 1, &StateMachine::callback_joy, this);
    sub_status = nh.subscribe(status_topic, 1,
                              &StateMachine::callback_status, this);
    sub_heartbeat = nh.subscribe(heartbeat_topic, 1, 
                                 &StateMachine::callback_heartbeat, this);
    sub_trigger = nh.subscribe(trigger_topic, 1, 
                                 &StateMachine::callback_trigger, this);
    
    // setup the states for start
    goal_state = move_to_mine;
    current_state = wait_to_start;

    // initialize the RobotCommand
    command.command.paddle_position = UP;
    command.command.paddle_status = OFF;
    command.command.bin_position = DOWN;
    command.header.frame_id = frame_id;
    command.header.seq = 0;

    // set waypoints from parameters
    ROS_INFO("Setting parameters (2)");
    std::vector<double> mine_x, mine_y, mine_theta, dump_x, dump_y, dump_theta;
    nh_.param<int>("waypoints/mine/num", num_mine_waypoints, 0);
    nh_.param<int>("waypoints/dump/num", num_dump_waypoints, 0);
    nh_.getParam("waypoints/mine/x", mine_x);
    nh_.getParam("waypoints/mine/y", mine_y);
    nh_.getParam("waypoints/mine/theta", mine_theta);
    nh_.getParam("waypoints/dump/x", dump_x);
    nh_.getParam("waypoints/dump/y", dump_y);
    nh_.getParam("waypoints/dump/theta", dump_theta);
    ROS_INFO("Set parameters (2)");

    if (num_mine_waypoints == 0 || num_dump_waypoints == 0)
    {
      ROS_ERROR("NO WAYPOINTS");
      goal_state = manual;
    }
    else if (mine_x.size() != num_mine_waypoints || 
             mine_y.size() != num_mine_waypoints ||
             mine_theta.size() != num_mine_waypoints ||
             dump_x.size() != num_dump_waypoints ||
             dump_y.size() != num_dump_waypoints ||
             dump_theta.size() != num_dump_waypoints)
    {
      ROS_ERROR("Wrong number of waypoints.");
      goal_state = manual;
    }
    else
    {
      mine_waypoints.resize(num_mine_waypoints);
      dump_waypoints.resize(num_dump_waypoints);
      for (int i = 0; i < num_mine_waypoints; i++)
      {
        geometry_msgs::Pose tmp;
        tmp.position.x = mine_x[i];
        tmp.position.y = mine_y[i];
        tmp.orientation = tf::createQuaternionMsgFromYaw(mine_theta[i]);
        mine_waypoints[i] = tmp;
      }
      for (int i = 0; i < num_dump_waypoints; i++)
      {
        geometry_msgs::Pose tmp;
        tmp.position.x = dump_x[i];
        tmp.position.y = dump_y[i];
        tmp.orientation = tf::createQuaternionMsgFromYaw(dump_theta[i]);
        dump_waypoints[i] = tmp;
      }
    }
    ROS_INFO("%d\n%d\n%f",num_mine_waypoints,num_dump_waypoints,
                          dump_waypoints[0].position.x);

    // loop until ros::Time::Now() != 0
    // necessary to get a valid timestamp on published messages
    while (ros::Time::now().sec == 0) {}
    last_heartbeat = ros::Time::now();
  }

  void callback_status(const IRIS_msgs::RobotStatus & robot_status) 
  {
    // TODO: implement states that aren't "wait_to_start" and "manual"
    switch (current_state)
    {
      // wait_to_start: do nothing
      case wait_to_start:
        command.command.cmd_vel = geometry_msgs::Twist();
        break;

      case localize:
        command.command.cmd_vel = geometry_msgs::Twist();
        break;

      case move_to_mine:
        break;

      case mine:
        break;

      case move_to_dump:
        break;

      case dump:
        break;

      // manual: do nothing
      case manual:
        break;
    }

    // publish the command
    command.header.stamp = ros::Time::now();
    command.header.seq++;
    pub_.publish(command);
  }

  void callback_joy(const sensor_msgs::Joy::ConstPtr & joy)
  {
    last_heartbeat = ros::Time::now();
    ROS_INFO("Received Joy\n");
    ROS_INFO("State = [%d]\n",current_state);
    
    if (joy->buttons[6])
    {
      current_state = manual;
      command.command.goal = geometry_msgs::Pose();
    }

    // only listen for commands other than switch to manual if in manual state
    if (current_state == manual)
    {
      if (joy->buttons[0])
      {
        command.command.bin_position = DOWN;
      }
      if (joy->buttons[1])
      {
        command.command.bin_position = UP;
      }
      if (joy->buttons[2])
      {
        command.command.paddle_position = DOWN;
      }
      if (joy->buttons[3])
      {
        command.command.paddle_position = UP;
      }
      if (joy->buttons[4])
      {
        command.command.paddle_status = OFF;
      }
      if (joy->buttons[5])
      {
        command.command.paddle_status = ON;
      }

      double x = joy->axes[1];
      double z = joy->axes[0];
      command.command.cmd_vel.linear.x = (x > -0.06 && x < 0.06) ? 0 : x/2;
      command.command.cmd_vel.angular.z = (z > -0.12 && z < 0.12) ? 0: z/2;
      ROS_INFO("[%f][%f]",x,z);

      // publish the command
      command.header.stamp = ros::Time::now();
      command.header.seq++;
      pub_.publish(command);
    }
  }

  void callback_cmdvel(const geometry_msgs::Twist & cmd_vel)
  {
    command.command.cmd_vel = cmd_vel;
  }

  void callback_trigger(const std_msgs::Bool & trigger)
  {
    if ((ros::Time::now() - last_heartbeat).toNSec() > timeout * msec2nsec)
    {
      ROS_INFO("No heartbeat. Stopping.");
      command.command.cmd_vel = geometry_msgs::Twist();
      command.command.paddle_status = 0;

      // publish the command
      command.header.stamp = ros::Time::now();
      command.header.seq++;
      pub_.publish(command);
    }
  }

  void callback_heartbeat(const std_msgs::Bool & beat)
  {
    ROS_INFO("Got Heartbeat");
    last_heartbeat = ros::Time::now();
    pub_.publish(command);
  }


private:
  // ROS node stuff
  ros::NodeHandle nh;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_joy;
  ros::Subscriber sub_status;
  ros::Subscriber sub_cmdvel;
  ros::Subscriber sub_heartbeat;
  ros::Subscriber sub_trigger;

  // state machine control
  state goal_state;
  state current_state;
  ros::Time last_heartbeat;
  int timeout;

  // the published command
  IRIS_msgs::RobotCommandStamped command;

  // reference frame for goals
  std::string frame_id;

  // topics subscribed and published
  std::string command_topic;
  std::string status_topic;
  std::string joy_topic;
  std::string heartbeat_topic;
  std::string trigger_topic;

  // postition waypoint variables
  int num_mine_waypoints;
  int num_dump_waypoints;
  std::vector<geometry_msgs::Pose> mine_waypoints;
  std::vector<geometry_msgs::Pose> dump_waypoints;
  int waypoint_counter;

}; // end of class StateMachine

int main(int argc, char **argv)
{
  // Initiate ROS
  ros::init(argc, argv, "finite_state_machine");

  // Create an object of class StateMachine that will take care of everything
  StateMachine machine;

  ros::spin();

  return 0;
}
