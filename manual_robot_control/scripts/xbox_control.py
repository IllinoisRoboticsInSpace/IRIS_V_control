#!/usr/bin/env python
import roslib
import io
roslib.load_manifest('manual_robot_control')
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(joy):
    msg = Twist()
    msg.linear.x = joy.axes[1]
    msg.angular.z = joy.axes[0]
    pub.publish(msg)


def main():
    
    rospy.init_node('manual_joy_cmd_vel')
    rospy.Subscriber('/IRIS/joy_filtered', Joy, callback)
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.spin()


if __name__== "__main__":
    main()
