#!/usr/bin/env python
import roslib
import io
roslib.load_manifest('serial_comm')
import rospy
import serial
import math
import time
from geometry_msgs.msg import Quaternion
from IRIS_msgs.msg import RobotCommandStamped
from IRIS_msgs.msg import RobotStatus
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler

'''
This is is mainly to test serial communication functionality with the
goal of sending motor commands. Dynamics are used to convert the
cmd_vel to two 8-bit motor speeds 0-255, where 128 is not moving.
'''
# variables for parameter
max_x_velocity = 0
use_serial = False
command_topic = ''
trigger_topic = ''
status_topic = ''
frame_id = ''
child_frame_id = ''
last_status = RobotStatus()
seq = 0

def callback_command(command):
    
    # convert the velocity commands to right and left tread speeds using
    # the robot physics
    linear = command.command.cmd_vel.linear.x
    angular = command.command.cmd_vel.angular.z
    R = 0.61
    left = (2*linear - R*angular) / 2
    right = (2*linear + R*angular) / 2

    
    # convert speeds to [-1000,1000] integer
    left = left / max_x_velocity * 1000.
    right = -right / max_x_velocity * 1000.
    if abs(left) > 1000:
        scale = abs(left) / 1000.
        left = left / scale
        right = right / scale
    if abs(right) > 1000: 
        scale = abs(right) / 1000.
        right = right / scale
        left = left / scale

    left = str(int(left))
    right = str(int(right))

    paddle_position = '1' if command.command.paddle_position else '0'
    bin_position = '1' if command.command.bin_position else '0'
    paddle_onoff = '1' if command.command.paddle_status else '0'

    thingtosend = (left + ',' + right + ',' + paddle_position + ',' + 
                   bin_position + ',' + paddle_onoff + '#' + '!')
    print('Sending: ', thingtosend)
    if use_serial:
        ser.write(thingtosend)
        ser.flushOutput()


def callback_trigger(trigger):
    global last_status
    global seq
    status = last_status

    if !use_serial:
        return

    # check if we got any data
    buffer_length = ser.inWaiting()
    print('Bytes in buffer: ', buffer_length)
    if buffer_length == 0: return

    # read serial data
    data_buffer = ser.read(buffer_length)
    ser.flushInput()
    
    # see if we got a valid message
    messages = data_buffer.split('!')
    good_data = False
    for i in reversed(range(len(messages))):
        message = messages(i)
        if message.endswith('#'):
            good_data = True
            break
    if !good_data: return

    # parse the message
    message = message[0:-1]
    status_data = message.split(',')
    status.odom.pose.pose.position.x = float(status_data[0])
    status.odom.pose.pose.position.y = float(status_data[1])
    orientation = Quaternion(*quaternion_from_euler(float(status_data[2])))
    status.odom.pose.pose.orientation = quaternion
    status.odom.pose.covariance[0] = float(status_data[3])
    status.odom.pose.covariance[7] = float(status_data[4])
    status.odom.pose.covariance[35] = float(status_data[5])
    status.odom.twist.twist.linear.x = float(status_data[6])
    status.odom.twist.twist.angular.z = float(status_data[7])
    status.bin_position = int(status_data[8])
    status.paddle_position = int(status_data[9])
    status.paddle_status = bool(status_data[10])

    # header stuff and publish
    status.odom.header.seq = seq
    seq = seq + 1
    status.odom.header.stamp = rospy.rostime.Time.now()
    last_status = status
    pub.publish(status)


def main():
    global max_x_velocity
    global command_topic
    global trigger_topic
    global use_serial
    global frame_id
    global child_frame_id
    global last_status
    global ser
    global pub

    rospy.init_node('serial_comm_python')
    max_x_velocity = rospy.get_param('~max_forward_velocity')
    use_serial = rospy.get_param('~use_serial')
    trigger_topic = rospy.get_param('~topic/trigger')
    command_topic = rospy.get_param('~topic/command')
    status_topic = rospy.get_param('~topic/status')
    frame_id = rospy.get_param('~frame_id/parent')
    child_frame_id = rospy.get_param('~frame_id/child')
    
    last_status.odom.header.frame_id = frame_id
    last_status.odom.child_frame_id = child_frame_id

    rospy.Subscriber(command_topic, RobotCommandStamped, callback_command)
    rospy.Subscriber(trigger_topic, Bool, callback_trigger)
    pub = rospy.Publisher(status_topic, RobotStatus, queue_size=1)
    
    if use_serial:
        ser = serial.Serial("/dev/ttyACM0", 115200, bytesize=serial.EIGHTBITS,
                             parity = serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE,
                             timeout=2,
                             xonxoff=0,
                             rtscts=0)
        print (ser.readline())

    rospy.spin()
    if use_serial:
        ser.close()

if __name__ == "__main__":
    main()
