#!/usr/bin/env python
import roslib
import io
roslib.load_manifest('serial_comm')
import rospy
import serial
import math
from IRIS_msgs.msg import RobotCommandStamped
from IRIS_msgs.msg import RobotStatusStamped


ser = serial.Serial("/dev/ttyACM0", 9600, bytesize=serial.EIGHTBITS,
                    parity = serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=4,
                    xonxoff=0,
                    rtscts=0)

def callback(command):
    
    linear = command.command.cmd_vel.linear.x
    print(linear)
    if linear > 0.49999: linear = 0.49999
    
    angular = command.command.cmd_vel.angular.z
    print(angular)
    if angular > 0.49999: angular = 0.49999

    lin_vel = chr(int(math.floor(256 * linear + 128)))
    print(lin_vel)
    ang_vel = chr(int(math.floor(256 * angular + 128)))
    print(ang_vel)

    motors = 0
    if command.command.paddle_position: motors = motors + 1
    if command.command.paddle_status: motors = motors + 2
    if command.command.bin_position: motors = motors + 4
    motorsch = chr(motors)

    print(motorsch)

    thingtosend = ('' + chr(int(linear))) + ('' +  chr(int(angular))) + ('' + chr(int(motors)))
    print(thingtosend)
    ser.write(thingtosend)
    ser.flushOutput()


def main():
    rospy.init_node('serial_comm_python')
    rospy.Subscriber('/IRIS/command', RobotCommandStamped, callback)
    rospy.spin()
    ser.close()

if __name__ == "__main__":
    main()
