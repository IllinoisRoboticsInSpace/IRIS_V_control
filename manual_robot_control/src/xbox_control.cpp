/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <sstream>

ros::Subscriber joy_sub_;
ros::Publisher chatter_pub;
double x_ = 0.0, z_ = 0.0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
   x_ = joy->axes[1];
   z_ = joy->axes[0];
   //Deadzone (because Joy's deadzone doesn't do anything for some reason):
   if (x_ > -0.06 && x_ < 0.06) x_ = 0;
   if (z_ > -0.12 && z_ < 0.12) z_ = 0; //For some reason horizontal axis on the xbox is really sensitive,
   //and can return values up to 0.1 without anything actively pushing on the joystick.
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "xbox_control");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

  while (ros::ok()) {
    geometry_msgs::Twist msg;
    
    msg.linear.x = x_;
    msg.angular.z = z_;
    
    chatter_pub.publish(msg);
    
    //ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
