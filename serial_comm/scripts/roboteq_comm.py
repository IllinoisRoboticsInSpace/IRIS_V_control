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

ser = serial.Serial("/dev/ttyUSB0", 115200, bytesize=serial.EIGHTBITS,
                     parity = serial.PARITY_NONE,
                     stopbits=serial.STOPBITS_ONE,
                     timeout=2,
                     xonxoff=0,
                     rtscts=0)

def callback_command(command):
    
    # convert the velocity commands to right and left tread speeds using
    # the robot physics
    linear = command.command.cmd_vel.linear.x
    angular = command.command.cmd_vel.angular.z
    left = ((linear + 0.307 * angular) / 2) # / 0.4035
    right = ((linear - 0.307 * angular) / 2) # / 0.4035
    
    # convert speeds to [-1000,1000] integer
    # this might be slightly off still
    left = int(left * 4000)
    right = int(-right * 4000)
    if abs(left) > 1000: 
        factor = float(abs(left)) / 1000
        left = int(left / factor)
        right = int(right / factor)
    if abs(right) > 1000:
        factor = float(abs(right)) / 1000
        left = int(left / factor)
        right = int(right / factor)

    left = '!G 2 ' + str(left) + '\r'
    right = '!G 1 ' + str(right) + '\r'

    print('Sending:\n' + left.encode('string_escape') + '\n' 
          + right.encode('string_escape') + '\n')
    ser.write(left)
    ser.write(right)
    ser.flushOutput()


def callback_trigger(trigger):
#   print("Got triggered")
    print('Bytes in buffer: ', ser.inWaiting())
    ack = ser.read(ser.inWaiting())
    print("Ack: " + ack + '\n')
    ser.flushInput()


def main():
    rospy.init_node('serial_comm_python')
    rospy.Subscriber('/IRIS/command', RobotCommandStamped, callback_command)
    rospy.Subscriber('/IRIS/serial_trigger', Bool, callback_trigger)
    rospy.spin()
    ser.close()

if __name__ == "__main__":
    main()
