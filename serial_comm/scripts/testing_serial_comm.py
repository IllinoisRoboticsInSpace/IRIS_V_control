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

ser = serial.Serial("/dev/ttyACM0", 115200, bytesize=serial.EIGHTBITS,
                     parity = serial.PARITY_NONE,
                     stopbits=serial.STOPBITS_ONE,
                     timeout=2,
                     xonxoff=0,
                     rtscts=0)
print (ser.readline())

def callback_command(command):
    
    # convert the velocity commands to right and left tread speeds using
    # the robot physics
    linear = command.command.cmd_vel.linear.x
    angular = command.command.cmd_vel.angular.z
    left = ((linear + 0.307 * angular) / 2) / 0.4035
    right = ((linear - 0.307 * angular) / 2) / 0.4035
    
    # convert speeds to [-1000,1000] integer
    # this might be slightly off still
    left = int(left * 1000)
    if left > 1000: left = 1000
    if left < -1000: left = -1000
    right = int(-right * 1000)
    if right > 1000: right = 1000
    if right < -1000: right = -1000

    left = str(left)
    right = str(right)

    paddle_position = '1' if command.command.paddle_position else '0'
    bin_position = '1' if command.command.bin_position else '0'
    paddle_onoff = '1' if command.command.paddle_status else '0'

    thingtosend = left + ',' + right + ',' + padde_position + ','
                  + bin_position + ',' + paddle_onoff + '!' 
    print(thingtosend)
    ser.write(thingtosend)
    ser.flushOutput()


def callback_trigger(trigger):
#   print("Got triggered")
    print('Bytes in buffer: ', ser.inWaiting())
    ack = ser.read(ser.inWaiting())
    print('Ack: ', ack)
    ser.flushInput()


def main():
    rospy.init_node('serial_comm_python')
    rospy.Subscriber('/IRIS/command', RobotCommandStamped, callback_command)
    rospy.Subscriber('/IRIS/serial_trigger', Bool, callback_trigger)
    rospy.spin()
    ser.close()

if __name__ == "__main__":
    main()
