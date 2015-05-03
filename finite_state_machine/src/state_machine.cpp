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
#include <IRIS_msgs/RobotStatusStamped.h>
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
   * Topics are hardcoded for now, but this should be fixed with parameters.
   * Subscribes to Joy command for mission control & RobotStatus for autonomy.
   **/
  StateMachine()
  {
    // topic to publish (command for the robot)
    pub_ = n_.advertise<IRIS_msgs::RobotCommandStamped>("/IRIS/command", 1);

    // topics to subscribe
    sub_joy = n_.subscribe("/joy_filtered", 1, &StateMachine::callback_joy, this);
    sub_status = n_.subscribe("/IRIS/status", 1,
                              &StateMachine::callback_status, this);
    sub_heartbeat = n_.subscribe("/IRIS/heartbeat", 1, 
                                 &StateMachine::callback_heartbeat, this);
    sub_trigger = n_.subscribe("/IRIS/FSM_trigger", 1, 
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

    // setup persistent data from parameters (once that's figured out)
    // hardwired for now
    frame_id = "1";
    mine_waypoints = std::vector<geometry_msgs::Pose>();
    dump_waypoints = std::vector<geometry_msgs::Pose>();
    max_timeout = 1000;
    
    // loop until ros::Time::Now() != 0
    // necessary to get a valid timestamp on published messages
    while (ros::Time::now().sec == 0) {}
    last_heartbeat = ros::Time::now();
  }

  void callback_status(const IRIS_msgs::RobotStatusStamped & robot_status) 
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
    if ((ros::Time::now() - last_heartbeat).toNSec() > max_timeout * msec2nsec)
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
  ros::NodeHandle n_;
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
  unsigned int max_timeout;

  // the published command
  IRIS_msgs::RobotCommandStamped command;

  // stuff needed for header
  std::string frame_id;

  // postition waypoint variables
  std::vector<geometry_msgs::Pose> mine_waypoints;
  std::vector<geometry_msgs::Pose> dump_waypoints;
  unsigned int waypoint_counter;

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
