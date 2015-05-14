#!/usr/bin/env python
import roslib
import io
roslib.load_manifest('serial_comm')
import rospy
import serial
import math
import time
from IRIS_msgs.msg import RobotCommandStamped
from IRIS_msgs.msg import RobotStatusStamped
from std_msgs.msg import Bool

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
#   print("Got triggered")
    if use_serial:
       print('Bytes in buffer: ', ser.inWaiting())
       ack = ser.read(ser.inWaiting())
       print("Ack: ")
       print(ack).encode('string_escape')
       ser.flushInput()


def main():
    global max_x_velocity
    global command_topic
    global trigger_topic
    global use_serial
    global ser
    rospy.init_node('serial_comm_python')
    max_x_velocity = rospy.get_param('~max_forward_velocity')
    use_serial = rospy.get_param('~use_serial')
    trigger_topic = rospy.get_param('~topic/trigger')
    command_topic = rospy.get_param('~topic/command')
    rospy.Subscriber(command_topic, RobotCommandStamped, callback_command)
    rospy.Subscriber(trigger_topic, Bool, callback_trigger)
    
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
