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
import math
import json

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

# Import curve spline  
path_to_spline = '../scripts/spline_pickle.pkl'
with open(path_to_spline, 'rb') as file:
    curve = pickle.load(file)
# boundary parameters 
s_boundaries = np.array([0.16728997170415225, 0.38172251444432276, 0.4547539216041566, 0.8032837862743827, 0.9823663123352225])
received_first_point = False
firstpoint = 0
currently_deccelerating = False
velocities =[]
saved_data = False
times = []
firsttime = 0
def callback(data, ser):
    global received_first_point, firstpoint, firsttime, currently_deccelerating, velocities, times, saved_data

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
    time_ros = data.header.stamp

    
    # print("data x", data.transform.translation.x)
    # for each point I want to set it relative to my complete run
    point = np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
    
    # print("point before ", point)
    point -= first_point

    point = rotation.apply(point)


    if not received_first_point:
        firstpoint = point
        firsttime = time_ros
        received_first_point = True
    else:
        d = np.linalg.norm(point - firstpoint)
        if d > 0.3:
            currently_deccelerating=True

    if currently_deccelerating:
        angle = 0
    else:
        angle = 90
    
    
    # get time from point
    # format = '%Y/%m/%d/%H:%M:%S.%f'
    # time = datetime.strptime(data.time, format)

    
    # # FIFO list buffer
    prev_points.append((point, time_ros))
    # only keep track of the last 4 points plus the current one
    if len(prev_points) > 5:
        prev_points.pop(0)
    #default value for velocioty
    velocity = 0
    # compute velocity for every point after the first 4 
    if len(prev_points) > 4:
        distance_traveled = 0
        for i in range(len(prev_points)-1):
            distance_traveled += np.linalg.norm(prev_points[i][0] - prev_points[i+1][0])
        total_time = (prev_points[4][1] - prev_points[0][1]).to_sec()
        velocity = distance_traveled/total_time
    
    if currently_deccelerating:
        velocities.append(velocity)
        times.append((time_ros - firsttime).to_sec())
    
    if currently_deccelerating and velocity < 0.1 and not saved_data:
        saved_data = True

        with open('list_vel', 'w') as file:
            json.dump([velocities, times], file)
    

    try:
        ser.write(pack('3B', 255, angle, 255))
        time.sleep(.01)

        

        
        # if dat!=b''and dat!=b'\r\n':
        #     try:
        #         dats=str(dat)
        #         dat1=dats.replace("b","")
        #         dat2=dat1.replace("'",'')
        #         dat3=dat2[:-4]
        #         data_list=ast.literal_eval(dat3)
        #         # check to see if data returned is correct
        #         if data_list[0] == 255 and data_list[-1] == 255:
        #             print("Correct:", data_list)
        #         else:
        #             print("Wrong data:", data_list)
        #         # print(dat3)
        #     except:
        #         print('Error in corvert, readed: ', dats)
        # time.sleep(0.1)
    except serial.SerialTimeoutException:
        ser.flush()
    except Exception as e:
        print(str(sys.exc_info())) #print error
    
def listener(ser):
    rospy.init_node('vicon_listener', anonymous=True)
    rospy.Subscriber("/vicon/slot_car/slot_car", TransformStamped, callback, (ser), queue_size =1)
    rospy.spin()

if __name__ == '__main__':
    try:
        ser=serial.Serial(baudrate='9600', timeout=.5, port='/dev/tty.usbmodem101', write_timeout=0.2)
    except:
        print('Port open error')
    listener(ser)