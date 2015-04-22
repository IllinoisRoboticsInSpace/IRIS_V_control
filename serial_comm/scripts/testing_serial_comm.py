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

def callback(command):
    
    # convert the velocity commands to right and left tread speeds using
    # the robot physics
    linear = command.command.cmd_vel.linear.x
    angular = command.command.cmd_vel.angular.z
    left = ((linear + 0.307 * angular) / 2) / 0.32675
    right = ((linear - 0.307 * angular) / 2) / 0.32675
    
    # convert speeds to 0-255 unisgned ascii char
    # this might be slightly off still
    left = (left + 1) / 2
    if left > 1: left = 1
    right = (right + 1) / 2
    if right > 1: right = 1
    left = math.floor(left*255)
    right = math.floor(right*255)

    paddle_position = '1' if command.command.paddle_position else '0'
    bin_position = '1' if command.command.bin_position else '0'
    paddle_status = '1' if command.command.paddle_status else '0'

    thingtosend = "%c%c%c%c%c" % (int(left),int(right),paddle_position,
                                  bin_position,paddle_status)
    print('Sent: [%d] [%d] [%c] [%c] [%c]' % (ord(thingtosend[0]),
                                        ord(thingtosend[1]),
                                        thingtosend[2],
                                        thingtosend[3],
                                        thingtosend[4]))
    if ser.isOpen(): print('serial is open')
    ser.write(thingtosend)
    ser.flushOutput()

    print("Bytes in buffer:", ser.inWaiting())
    ack = ser.read(ser.inWaiting())
    print(ack)
    ser.flushInput()
    print("Ack:  [%d] [%d] [%c] [%c] [%c]" % (ord(ack[0]),
                                        ord(ack[1]),
                                        ack[2],
                                        ack[3],
                                        ack[4]))


def main():
    rospy.init_node('serial_comm_python')
    rospy.Subscriber('/IRIS/command', RobotCommandStamped, callback)
    rospy.spin()
    ser.close()

if __name__ == "__main__":
    main()
