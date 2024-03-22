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

#********** Returns True if point is in segment  **********
def is_in_segment(point, segment):
    if segment[1,0] <= point[0] and point[0] <= segment[0,0] and segment[1,1] <= point[1] and point[1] <= segment[0,1]:
        return True 
    else: 
        return False
    

#********** Create list to store previous points in  **********
prev_points = []

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

    #import rotation pickle and first point 
    with open('rotation_pickle.pkl', 'rb') as file:
        rotation, first_point, segments = pickle.load(file)

    # for each point I want to set it relative to my complete run
    point = np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
    point -= first_point
    point = rotation.apply(point)

    #check which point segment is in 
    current_segment = -1
    for i, segment in enumerate(segments):   
        if is_in_segment(point, segment):
            current_segment = i
            break
    if current_segment == -1:
        print("No segment found for point")
    
    # get time from point
    format = '%Y/%m/%d/%H:%M:%S.%f'
    time = datetime.strptime(data.time, format)


    # FIFO list buffer
    prev_points.append((point, time))
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
        total_time = (prev_points[4][1] - prev_points[0][1]).total_seconds()
        velocity = distance_traveled/total_time
    
    # check whether velocity is safe for segment 
    safe_ranges = [2.1043911825424666, 1.8191871053204132, 2.4426514688135, 2.54147152599213, 2.1777409348338157]
    #CONTROLLER 
    max_velocity_for_segment = safe_ranges[current_segment]
    #tune constant 
    kp = 1
    # kp(vmax - vcurr)
    control_input = kp * (max_velocity_for_segment - velocity)

    # now i have slope and intercept 
    # copy and paste the slope and itnerfcept from experiment 
    slope = 0
    intercept = 0
    
    #compute the control input 
    set_angle = (max_velocity_for_segment - intercept) / slope 
    if velocity >= max_velocity_for_segment:
        angle = set_angle
    else:
        pass
        #Duy 
        #pass human input 





    
    


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