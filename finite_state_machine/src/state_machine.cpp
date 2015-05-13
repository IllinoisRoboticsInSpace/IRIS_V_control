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

// square function
inline double pow2(double a) {return a * a;}

// list of states for machine
enum state {wait_to_start, localize, move_to_mine, mine, move_to_dump,
                dump, manual};
enum actuator_status {retracting, down, extending, up};
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
    nh_.param<std::string>("topic/command", command_topic, "/IRIS/command");
    nh_.param<std::string>("topic/status", status_topic, "/IRIS/status");
    nh_.param<std::string>("topic/joy", joy_topic, "/IRIS/joy_filtered");
    nh_.param<std::string>("topic/beat", heartbeat_topic, "/IRIS/heartbeat");
    nh_.param<std::string>("topic/trigger", trigger_topic, "/IRIS/FSM_trigger");
    nh_.param<std::string>("frame_id", frame_id, "0");
    nh_.param<int>("timeout", timeout, 1000);
    nh_.param<double>("tolerance/error/position", position_error_tol, 0.1);
    nh_.param<double>("tolerance/error/orientation", orientation_error_tol, 0.1);
    nh_.param<double>("tolerance/goal/position", position_goal_tol, 0.1);
    nh_.param<double>("tolerance/goal/orientation", orientation_goal_tol, 0.1);

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
    std::vector<double> mine_x, mine_y, mine_theta, dump_x, dump_y, dump_theta;
    nh_.param<int>("waypoints/mine/num", num_mine_waypoints, 0);
    nh_.param<int>("waypoints/dump/num", num_dump_waypoints, 0);
    nh_.getParam("waypoints/mine/x", mine_x);
    nh_.getParam("waypoints/mine/y", mine_y);
    nh_.getParam("waypoints/mine/theta", mine_theta);
    nh_.getParam("waypoints/dump/x", dump_x);
    nh_.getParam("waypoints/dump/y", dump_y);
    nh_.getParam("waypoints/dump/theta", dump_theta);

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
    waypoint_counter = 0;

    // loop until ros::Time::Now() != 0
    // necessary to get a valid timestamp on published messages
    while (ros::Time::now().sec == 0) {}
    last_heartbeat = ros::Time::now();
  }

  void callback_status(const IRIS_msgs::RobotStatus & status) 
  {
    bool done = false;
    while (!done)
    {
      switch (current_state)
      {
        // wait_to_start: do nothing
        case wait_to_start:
          command.command.cmd_vel = geometry_msgs::Twist();
          done = true;
          break;
  
        // localize: stay in place until good position data
        case localize:
          if (errorTooBig(status.odom.pose))
          {
            command.command.cmd_vel = geometry_msgs::Twist();
            done = true;
          }
          else
          {
            current_state = goal_state;
          }
          break;
  
        // move_to_mine: move to the mining area (mine_waypoints[0])
        case move_to_mine:
          if (status.bin_position != down)
          {
            command.command.goal = status.odom.pose.pose;
            command.command.bin_position = DOWN;
            done = true;
          }
          else if (errorTooBig(status.odom.pose))
          {
            goal_state = move_to_mine;
            current_state = localize;
          }
          else if (reachedGoal(status.odom.pose.pose))
          {
            current_state = mine;
            waypoint_counter = 1;
          }
          else
          {
            command.command.goal = mine_waypoints[0];
            done = true;
          }
          break;

        // mine: activate and lower paddle, drive through waypoints
        case mine:
          if (waypoint_counter == num_mine_waypoints)
          {
            current_state = move_to_dump;
            waypoint_counter = 0;
          }
          else if (status.paddle_position != down || status.paddle_status != ON)
          {
            command.command.goal = status.odom.pose.pose;
            command.command.paddle_position = DOWN;
            command.command.paddle_status = ON;
            done = true;
          }
          else if (errorTooBig(status.odom.pose))
          {
            goal_state = mine;
            current_state = localize;
          }
          else if (reachedGoal(status.odom.pose.pose))
          {
            waypoint_counter++;
          }
          else
          {
            command.command.goal = mine_waypoints[waypoint_counter];
            done = true;
          }
          break;
        
        // move_to_dump: stop and raise the paddle, drive to the bin
        case move_to_dump:
          if (waypoint_counter == num_dump_waypoints)
          {
            current_state = dump;
            waypoint_counter = 0;
          }
          else if (status.paddle_position != up || status.paddle_status != OFF)
          {
            command.command.goal = status.odom.pose.pose;
            command.command.paddle_position = UP;
            command.command.paddle_status = OFF;
            done = true;
          }
          else if (errorTooBig(status.odom.pose))
          {
            goal_state = move_to_dump;
            current_state = localize;
          }
          else if (reachedGoal(status.odom.pose.pose))
          {
            waypoint_counter++;
          }
          else
          {
            command.command.goal = dump_waypoints[waypoint_counter];
            done = true;
          }
          break;
  
        // dump: raise the bin to dump
        case dump:
          if (status.bin_position != UP)
          {
            command.command.bin_position = UP;
            command.command.goal = status.odom.pose.pose;
            done = true;
          }
          else
          {
            current_state = move_to_mine;
          }
          break;
  
        // manual: do nothing
        case manual:
          break;
      }
    }
    // publish the command
    command.header.stamp = ros::Time::now();
    command.header.seq++;
    pub_.publish(command);
    
  }

  /* set states/commands based on joy msg */
  void callback_joy(const sensor_msgs::Joy::ConstPtr & joy)
  {
    last_heartbeat = ros::Time::now();
    ROS_INFO("Received Joy\n");
    ROS_INFO("State = [%d]\n",current_state);
    
    if (joy->buttons[6])
    {
      current_state = manual;
    }

    if (joy->buttons[7] && current_state == wait_to_start)
    {
      current_state = localize;
    }

    // only listen for other commands if in manual state 
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

  /* set command velocity from move_base, as long as not manual*/
  void callback_cmdvel(const geometry_msgs::Twist & cmd_vel)
  {
    if (current_state != manual)
    {
      command.command.cmd_vel = cmd_vel;
    }
  }

  /* regularly check if got heartbeat and stop if we lost connection */
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

  /* heartbeat for ensuring we are still connected to mission control */
  void callback_heartbeat(const std_msgs::Bool & beat)
  {
    last_heartbeat = ros::Time::now();
    pub_.publish(command);
  }

  /* check if the localization error is greater than the tolerance */
  bool errorTooBig(geometry_msgs::PoseWithCovariance localization)
  {
    double varX = localization.covariance[0];
    double varY = localization.covariance[7];
    double varTheta = localization.covariance[35];

    return (varX > pow2(position_error_tol) || 
            varY > pow2(position_error_tol) ||
            varTheta > pow2(orientation_error_tol));
  }

  /* check if reached the goal position */
  bool reachedGoal(const geometry_msgs::Pose & localization)
  {
    double errorX = localization.position.x - command.command.goal.position.x;
    double errorY = localization.position.y - command.command.goal.position.y;
    double errorTheta = tf::getYaw(localization.orientation) -
                        tf::getYaw(command.command.goal.orientation);

    return (pow2(errorX) + pow2(errorY) < pow2(position_goal_tol) &&
            pow2(errorTheta) < pow2(orientation_error_tol));
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

  // error tolerances
  double position_error_tol;
  double orientation_error_tol;
  double position_goal_tol;
  double orientation_goal_tol;

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
