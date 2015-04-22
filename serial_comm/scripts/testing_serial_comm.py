'''
This is is mainly to test serial communication functionality with the
goal of sending motor commands. Dynamics are used to convert the
cmd_vel to two 8-bit motor speeds 0-255, where 128 is not moving.
'''
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


ser = serial.Serial("/dev/ttyACM0", 9600, bytesize=serial.EIGHTBITS,
                     parity = serial.PARITY_NONE,
                     stopbits=serial.STOPBITS_ONE,
                     timeout=4,
                     xonxoff=0,
                     rtscts=0)
print (ser.readline())

def callback(command):
    
    linear = command.command.cmd_vel.linear.x
    angular = command.command.cmd_vel.angular.z
    left = ((linear + 0.307 * angular) / 2) / 0.32675
    right = ((linear - 0.307 * angular) / 2) / 0.32675

    left = (left + 1) / 2
    if left > 1: left = 1
    right = (right + 1) / 2
    if right > 1: right = 1
    right = right
    left = math.floor(left*255)
    right = math.floor(right*255)
    print("left: ", left)
    print("right: ", right)




    motors = 0
    paddle_position = command.command.paddle_position
    bin_position = command.command.bin_position
    paddle_status = command.command.paddle_status

    thingtosend = "%c%c%c%c%c" % (int(left),int(right),paddle_position,
                                  bin_position,paddle_status)
    print(thingtosend)
    print(len(thingtosend))
    print("[%d] [%d] [%d] [%d] [%d]" % (ord(thingtosend[0]),
                                        ord(thingtosend[1]),
                                        ord(thingtosend[2]),
                                        ord(thingtosend[3]),
                                        ord(thingtosend[4])))
    if ser.isOpen(): print("serial is open")
    ser.write(thingtosend)
    ser.flushOutput()
    print(ser.readline())


def main():
    rospy.init_node('serial_comm_python')
    rospy.Subscriber('/IRIS/command', RobotCommandStamped, callback)
    rospy.spin()
    ser.close()

if __name__ == "__main__":
    main()
