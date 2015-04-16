// standard library includes
#include <cmath>
#include <string>

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
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

// define pi for math use
const double pi = 4 * atan(1.);

// list of states for machine
enum state_t = {wait_to_start, localize, move_to_min, mine, move_to_dump,
                dump, manual};

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

    
