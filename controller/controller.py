import serial
from struct import *
import sys
import time
import random
import ast
import rospy
from geometry_msgs.msg import TransformStamped
import socket

angle = 0

def callback(data, ser):
    global angle
    # get the car pose
    # rospy.loginfo("{:.3f}, {:.3f}, {:.3f}".format(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z))
    '''
    Get car pose with the following syntax
    x = data.transform.translation.x
    y = data.transform.translation.y
    z = data.transform.translation.z
    rx = x = data.transform.rotation.x
    ry = x = data.transform.rotation.y
    rz = x = data.transform.rotation.z
    rw = x = data.transform.rotation.w
    '''
    # print("x:", data.transform.translation.x, "y:", data.transform.translation.y, "z:", data.transform.translation.z)
    # save first point
    # subtract first point from eahc point adn apply rotation from pickle object and then with the copy pasted isinsegment then
    # i am gonna check what segment its in and then compare it to safe range and then clip . 
    # for velocity keep a buffer of the last ten time steps and keep list of last ten 

    # calculate the velocity based on the received vicon data
    velocity = 0.0

    # process the targeted velocity and send the angle to the Arduino via serial
    angle = 0

    try:
        # the first and last number are used to check if the returned data is correct, 127
        # the 8 other values in between are dynamic data
        ser.write(pack('3B', 255, angle, 255))
        time.sleep(.01)
        dat=ser.readline()
        
        if dat!=b''and dat!=b'\r\n':
            try:
                dats=str(dat)
                dat1=dats.replace("b","")
                dat2=dat1.replace("'",'')
                dat3=dat2[:-4]
                data_list=ast.literal_eval(dat3)
                # check to see if data returned is correct
                if data_list[0] == 255 and data_list[-1] == 255:
                    print("Correct:", data_list)
                else:
                    print("Wrong data:", data_list)
                # print(dat3)
            except:
                print('Error in corvert, readed: ', dats)
        time.sleep(0.1)
    except:
        print(str(sys.exc_info())) #print error
    
def listener(ser):
    rospy.init_node('vicon_listener', anonymous=True)
    rospy.Subscriber("/vicon/slot_car/slot_car", TransformStamped, callback, (ser))
    rospy.spin()

if __name__ == '__main__':
    try:
        ser=serial.Serial(baudrate='9600', timeout=.5, port='/dev/tty.usbmodem101')
    except:
        print('Port open error')
    listener(ser)