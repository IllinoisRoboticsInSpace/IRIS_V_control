#!/usr/bin/env python
import roslib
import io
roslib.load_manifest('serial_comm')
import rospy
import roslib
import serial
from geometry_msgs.msg import Twist
from time import sleep
from std_msgs.msg import Bool
ser = serial.Serial("/dev/ttyACM0", 9600, bytesize=serial.EIGHTBITS,
                     parity=serial.PARITY_NONE,
                     stopbits=serial.STOPBITS_ONE,
                     timeout=4,
                     xonxoff=0,
                     rtscts=0)
                     
#print ("Done opening shit")
#ser.setDTR(False)
#sleep(1)
# toss any data already received, see
# http://pyserial.sourceforge.net/pyserial_api.html#serial.Serial.flushInput
#ser.flushInput()
#ser.setDTR(True)

last_command = -2
last_maxxon = -2
flag = False
def callback(data):

    global last_command, last_maxxon, flag
    #send the initialization command to the serial controller only once before we send the maxxon or the actuator command 
    if flag:
        print("done initializing")
        ser.write("LT#")
        ser.flushOutput()
        flag = False
   


    
    #SENDING THE ACTUATOR COMMAND 
    #get the command paramenter and set the respective command for the actuator
    command = rospy.get_param('/bin/command')
    if (last_command == command):
    	print("last_command = command ", command)
    else:
    	if command == 1:
        	cmd_sent = 'E'
    	elif command == 0:
        	cmd_sent = 'H'
    	elif command == -1:
        	cmd_sent = 'R'


    #send the cmd to the actuator
    #rospy.sleep(1)
    	print ("cmd sent", cmd_sent)
    	print ("sending actuator cmd")
    	ser.write('L' + cmd_sent + '#')
    	ser.flushOutput()
    
    

    #THE MAXON CODE 

 #get the turbine command and set the maxxon controller command, send the maxxon cmd to the maxxon controller
    maxxon_cmd = rospy.get_param('/bin/turbine')
    ser.flushOutput()
    if (last_maxxon == maxxon_cmd):
    	print()
    else:
    	if maxxon_cmd == 1:
    	    maxxon_sent = 1
    	elif maxxon_cmd == 0:
            maxxon_sent = 0
    	print("sending maxxon ctrlr cmd", maxxon_sent)
        if maxxon_sent:
        #rospy.sleep(1)
            rospy.sleep(1)
            ser.write('M11#')
        else:
        #rospy.sleep(1)
            rospy.sleep(1)
            ser.write('M00#')
        #rospy.sleep(1)   
        ser.flushOutput()
    #read for a READY string from the arduino
    #serin = ser.readline()


    #rospy.sleep(1)
    #don't send the LT# for this version of the arduino - Motor shield R3
    	
    

    #serin = ser.read(10)
    #print (serin)
    #ser.flushInput()
    #serin = ser.read(10)
    #print (serin, ser.inWaiting())
    #ser.flushInput()

    
   
    
    
    '''
    '''
    #print (serin)
    #ser.flush()
    #rospy.sleep(1)
    #serin = ser.readline()
    #print (serin)
    #ser.flush()
    
    
    '''
   
    ser.flushOutput()
    ser.flushInput()
    rospy.sleep(5)
    print ("testing LC crapp")
    ser.write(serial.to_bytes('LC#'))
    #print(ser.outWaiting())
    #rospy.sleep(1)
    #ser.flushInput()
    
    serin = ser.read(30)
    #ser.flush()
    print (serin, ser.inWaiting())
    #ser.flushInput()
    if (serin == "READY\r\n"):
    	print("got the ready crapp")
     	rospy.set_param('/bin/position', command)
    ser.flushInput()
    #serin = ser.readline()
    #print (serin)
    #ser.flushInput()
    #print ("before the lc bs ", serin)
    #serin = ser.readline()
    #print ("stuff i received from lc# bs ", serin)
    
    #ser.flushInput()
    '''
    last_command = command
    last_maxxon = maxxon_cmd
def main():
    rospy.init_node('serial_tester')
    #r = rospy.Rate(2)
    rospy.Subscriber('state_machine_trigger',Bool,callback)
    rospy.spin()
    ser.close()

if __name__ == "__main__":
    main()

















'''
    counter = 0
    a = "LS#"
    flag = True
    while counter < 20:
        sleep(1)    
        ser.write(a)
        sleep(1)
        ser.flush()
        serin = ser.readline()
        
        #if (counter < 2):
        #    ser.write("hello")
        #    counter +=1
        #    continue
        print (serin, counter)
        #print (serin)
        
        ser.flush()
        if (counter%2 == 0):
            a = "M111#"
        else:
            a = "LS#"
            #"
        #print("done writing")
        #ser.write('111')
        #ser.write('1')
        #ser.write('1')
        #sleep(0.1)
       
        #serin = ser.readline()
        
        #print (serin)#len(serin))
        counter+=1
        serin = ""
        ser.flush()
    
    #ser.close()

    
    connected = False

    while not connected:
        ser.write('a')
        serin = ser.readline()
        #sleep(0.1)
        connected = True
        print (serin)

    


    #while ser.read() == '1':
     #  temp = ser.readline()
        #print(temp)

    ser.close()
    #rospy.spin()


'''



#THE MAXON CODE 
'''
 #get the turbine command and set the maxxon controller command, send the maxxon cmd to the maxxon controller
    maxxon_cmd = rospy.get_param('/bin/turbine')
    ser.flushOutput()
    if (last_maxxon == maxxon_cmd):
    	print()
    else:
    	if maxxon_cmd == 1:
    	    maxxon_sent = 1
    	elif maxxon_cmd == 0:
            maxxon_sent = 0
    	print("sending maxxon ctrlr cmd", maxxon_sent)
        if maxxon_sent:
        #rospy.sleep(1)
            rospy.sleep(1)
            ser.write('M11#')
        else:
        #rospy.sleep(1)
            rospy.sleep(1)
            ser.write('M00#')
        #rospy.sleep(1)   
        ser.flushOutput()
    #read for a READY string from the arduino
    #serin = ser.readline()

'''
