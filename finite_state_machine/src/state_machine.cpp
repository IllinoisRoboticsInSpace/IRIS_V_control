// standard library includes
#include <cmath>
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
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
#include <std_msgs/Header.h>

// define pi for math use
const double pi = 4 * atan(1.);

// list of states for machine
enum state_t = {wait_to_start, localize, move_to_min, mine, move_to_dump,
                dump, manual};
enum actuator_state = {rectracting, down, extending, up};
enum actuator_command : bool {DOWN = false, UP = true};
enum paddle_state : bool {OFF = false, ON = true};

/** 
 * class to maintain shared and persistent data whie subscribing to
 * multiple topics and publishing the RobotCommand
 **/
class SubscribeAndPublish()
{

public:
  /**
   * SubscribeAndPublish constructor
   * Topics are hardcoded for now, but this should be fixed with parameters.
   * Subscribes to Joy command for mission control & RobotStatus for autonomy.
   **/
  SubscribeAndPublish()
  {
    // topic to publish (command for the robot)
    pub_ = n_.advertise<IRIS_msgs::RobotCommandStamped>("/IRIS/command", 1);

    // topics to subscribe
    sub_joy = n_.subscribe("/joy", 1, &SubscribeAndPublish::callback_joy, this);
    sub_status = n_.subscribe("/IRIS/status", 1,
                              &SubscribeAndPublish::callback_status, this);
    
    // setup the states for start
    goal_state = move_to_mine;
    current_state = wait_to_start;

    // initialize the RobotCommand
    command.command.paddle_position = UP;
    command.command.paddle_state = OFF;
    command.command.bin_position = DOWN;
    command.command.active_goal = false;
    command.header.frame_id = frame_id;
    command.header.seq = 0;

    // setup persistent data from parameters (once that's figured out)
    // hardwired for now
    frame_id = "1";
    
    // loop until ros::Time::Now() != 0
    // necessary because of how nodes are intialized, or something like that
    while (ros::Time::now().sec == 0) {}
  };

  void callback_status(const IRIS_msgs::RobotStatusStamped & robot_status) 
  {
    // TODO: implement states that aren't "wait_to_start" and "manual"
    switch (current_state)
    {
      // wait_to_start: do nothing
      case wait_to_start:
        break;

      // manual: do nothing
      case manual:
        break;
    }

    // publish the command
    command.header.stamp = ros::Time::now();
    command.header.seq++;
    pub_.publish(command);
  };

  void callback_joy(const sensor_msgs::Joy::ConstPtr & joy)
  {
    if (joy->buttons[0])
    {
      current_state = manual;
      command.command.active_goal = false;
      command.command.goal = geometry_msgs::Pose();
    }
  }

private:
  // ROS node stuff
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_joy;
  ros::Subscriber sub_status;

  // state machine control
  state_t goal_state;
  state_t current_state;

  // the published command
  IRIS_msgs::RobotCommandStamped command;

  // stuff needed for header
  std::string frame_id;

  // postition waypoint variables
  std::vector<geometry_msgs::Pose> mine_waypoints;
  std::vector<geometry_msgs::Pose> dump_waypoints;
  unsigned int waypoint_counter;

} // end of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "finite_state_machine");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
