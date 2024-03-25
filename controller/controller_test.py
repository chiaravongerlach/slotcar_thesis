import serial
import pickle
from struct import *
import sys
import time
import random
import ast
import rospy
from geometry_msgs.msg import TransformStamped
import socket
import numpy as np
from datetime import datetime

#********** Create Segments  **********
"""
Break track into four segments that we can find safe velocity ranges for
Convention: first point is top right and then bottom left 
"""
# First Line 
line_1 = np.array([[0.5594, 1.1028], [0.4208, 0.4009]]) 
# Turn 1
turn_1 = np.array([[line_1[0, 0], line_1[1, 1]], [0.0234, 0.0405]])
# Line 2
line_2 = np.array([[0.1993, 0.6942],[turn_1[1,0], line_1[1, 1]]])
# Loop
loop = np.array([[line_2[0,0],line_1[0,1]],[-0.0645, line_2[0,1]]])
# Second Turn
turn_2 = np.array([[line_1[0,0], 1.4731], [loop[1,0], line_1[0,1]]])

# Set relative to starting point
start_point = np.array([0.52252043, 1.04219351])
line_1 -= start_point
turn_1 -= start_point
line_2 -= start_point
loop -= start_point
turn_2 -= start_point

# Use this to iterature through the segments later on when you wanna check what point crashed 
segments = [line_1, turn_1, line_2, loop, turn_2]

#********** Returns True if point is in segment  **********
def is_in_segment(point_t, segment):
    if segment[1,0] <= point_t[0] and point_t[0] <= segment[0,0] and segment[1,1] <= point_t[1] and point_t[1] <= segment[0,1]:
        return True 
    else: 
        return False
    

#********** Create list to store previous points in  **********
prev_points = []
#import rotation pickle and first point 
with open('rotation_pickle.pkl', 'rb') as file:
    rotation, first_point, segments = pickle.load(file)




def callback(data, ser):
    rospy.loginfo("{:.3f}, {:.3f}, {:.3f}".format(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z))
    print("data x", data.transform.translation.x)
    # for each point I want to set it relative to my complete run
    point = np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
    
    print("point before ", point)
    point -= first_point

    point = rotation.apply(point)
    print("point with rotation", point)


    #check which point segment is in 
    current_segment = -1
    for i, segment in enumerate(segments):   
        if is_in_segment(point, segment):
            current_segment = i
            break
    if current_segment == -1:
        print("No segment found for point")

    
    if current_segment == 0:
        angle = 35
    else:
        angle = 0
    print(current_segment)

    try:
        # the first and last number are used to check if the returned data is correct, 127
        # the 8 other values in between are dynamic data
        ser.write(pack('3B', 255, angle, 255))
        time.sleep(.1)
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
    except serial.SerialTimeoutException:
        ser.flush()
    except:
        print(str(sys.exc_info())) #print error

    
def listener(ser):
    rospy.init_node('vicon_listener', anonymous=True)
    rospy.Subscriber("/vicon/slot_car/slot_car", TransformStamped, callback, (ser), queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        ser=serial.Serial(baudrate='9600', timeout=.5, port='/dev/tty.usbmodem1101', write_timeout=0.2)
    except:
        print('Port open error')
    listener(ser)