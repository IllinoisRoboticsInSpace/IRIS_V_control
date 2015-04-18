#!/usr/bin/env python
import roslib
roslib.load_manifest('serial_comm')
import rospy
from std_msgs.msg import Bool



def dummy_publisher():
    pub = rospy.Publisher('state_machine_trigger', Bool)
    rospy.init_node('dummy_node_state_machine', anonymous=True)
    r = rospy.Rate(5) # 2hz
    while not rospy.is_shutdown():
        data = True
        pub.publish(data)
        r.sleep()


if __name__ == '__main__':
    try:
        dummy_publisher()
    except rospy.ROSInterruptException: pass




