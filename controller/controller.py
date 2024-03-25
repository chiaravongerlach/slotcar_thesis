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

    # get the car pose
    rospy.loginfo("{:.3f}, {:.3f}, {:.3f}".format(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z))
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
        
    
    # # get time from point
    # format = '%Y/%m/%d/%H:%M:%S.%f'
    # time = datetime.strptime(data.time, format)


    # # FIFO list buffer
    # prev_points.append((point, time))
    # # only keep track of the last 4 points plus the current one
    # if len(prev_points) > 5:
    #     prev_points.pop(0)
    # #default value for velocioty
    # velocity = 0
    # # compute velocity for every point after the first 4 
    # if len(prev_points) > 4:
    #     distance_traveled = 0
    #     for i in range(len(prev_points)-1):
    #         distance_traveled += np.linalg.norm(prev_points[i][0] - prev_points[i+1][0])
    #     total_time = (prev_points[4][1] - prev_points[0][1]).total_seconds()
    #     velocity = distance_traveled/total_time
    
    # # check whether velocity is safe for segment 
    # safe_ranges = [2.1043911825424666, 1.8191871053204132, 2.4426514688135, 2.54147152599213, 2.1777409348338157]
    # #CONTROLLER 
    # max_velocity_for_segment = safe_ranges[current_segment]
    # #tune constant 
    # kp = 1
    # # kp(vmax - vcurr)
    # control_input = kp * (max_velocity_for_segment - velocity)

    # # now i have slope and intercept 
    # # copy and paste the slope and itnerfcept from experiment 
    # slope = 0
    # intercept = 0
    
    # #compute the control input 
    # set_angle = (max_velocity_for_segment - intercept) / slope 
    # if velocity >= max_velocity_for_segment:
    #     angle = set_angle
    # else:
    #     pass
    #     #Duy 
    #     #pass human input 
    
    # if current_segment == 0:
    #     angle = 50
    # else:
    #     angle = 0
    # print(current_segment)
    print(current_segment)
    angle = 180
    







    
    


    # print("x:", data.transform.translation.x, "y:", data.transform.translation.y, "z:", data.transform.translation.z)
    # save first point
    # subtract first point from eahc point adn apply rotation from pickle object and then with the copy pasted isinsegment then
    # i am gonna check what segment its in and then compare it to safe range and then clip . 
    # for velocity keep a buffer of the last ten time steps and keep list of last ten 

    # calculate the velocity based on the received vicon data
   

    # process the targeted velocity and send the angle to the Arduino via serial
    

    try:
        # the first and last number are used to check if the returned data is correct, 127
        # the 8 other values in between are dynamic data
        ser.write(pack('3B', 255, angle, 255))
        time.sleep(.01)
        # dat=ser.readline()
        
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
    except:
        print(str(sys.exc_info())) #print error
    
def listener(ser):
    rospy.init_node('vicon_listener', anonymous=True)
    rospy.Subscriber("/vicon/slot_car/slot_car", TransformStamped, callback, (ser), queue_size =1)
    rospy.spin()

if __name__ == '__main__':
    try:
        ser=serial.Serial(baudrate='9600', timeout=.5, port='/dev/tty.usbmodem1101', write_timeout=0.2)
    except:
        print('Port open error')
    listener(ser)
