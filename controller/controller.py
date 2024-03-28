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

def is_in_segment(point_t, segment):
    '''Returns True if point is in the rectangle containing a segment of the track. '''
    if segment[1,0] <= point_t[0] and point_t[0] <= segment[0,0] and segment[1,1] <= point_t[1] and point_t[1] <= segment[0,1]:
        return True 
    else: 
        return False

def get_segment(s_point, s_boundaries):
    ''' Returns the spline segment a curve parameter belongs to.'''
    for i, endpoint in enumerate(s_boundaries):
        if endpoint > s_point:
            return i
    return 0  #  handle the corner case in which s_point is past the last endpoint
    

#********** Create list to store previous points in  **********
prev_points = []
control_times = []
#import rotation pickle and first point 
with open('rotation_pickle.pkl', 'rb') as file:
    rotation, first_point, segments = pickle.load(file)

# Import curve spline  
path_to_spline = '../scripts/spline_pickle.pkl'
with open(path_to_spline, 'rb') as file:
    curve = pickle.load(file)
# boundary parameters 
# s_boundaries = np.array([0.16728997170415225, 0.38172251444432276, 0.4547539216041566, 0.8032837862743827, 1.0)
s_boundaries = np.array([0.16728997170415225, 0.38172251444432276, 0.4547539216041566, 0.8032837862743827, 0.9823663123352225])
# s_boundaries = np.array([0.16728997170415225, 0.38172251444432276, 0.5400, 0.8032837862743827, 0.9823663123352225])
n_segments = len(s_boundaries)

# Note: the last segment (which really ends at 1.0) slightly overlaps with the first stegment (which starts at 0.0).
# to avoid length miscalculations, we set the endpoint at ~0.98, where the first segement begins.
# If a point gets mapped to s in [~0.98, 1.0), we need to remap it to the first segment.

def distance_along_curve(s_start, s_end, curve):
    ''' Returns the distance to the next segment along a periodic spline curve.

    Automatically handles slight overlaps between the end and start of the spline.
    '''
    if s_end > s_start:
        spline_to_boundary = curve.windowCurve(s_start, s_end) 
        distance_to_next_segment = spline_to_boundary.getLength()
    else:  # handle the odd case where we need to switch from the end to the beginning of the spline.
        spline_to_boundary = curve.windowCurve(s_start, 1) 
        distance_to_curve_end = spline_to_boundary.getLength()
        spline_to_boundary = curve.windowCurve(0, s_end) 
        distance_from_curve_start = spline_to_boundary.getLength()
        distance_to_next_segment = distance_from_curve_start - distance_to_curve_end
    return distance_to_next_segment

def callback(data, ser):

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
    # maximum and minimum acceleration commands available to the safety filter
    max_acc = 5.1869433048767375
    max_decc = -5.979429307875425

    # velocity constraints in each segment
    # safe_ranges = [2.1043911825424666, 1.8191871053204132, 2.4426514688135, 2.54147152599213, 2.1777409348338157]
    max_speeds = [2.1043911825424666, 1.8191871053204132, 2.4426514688135, 3., 2.1777409348338157]
    # min_speeds = [0., 0., 0., 2.54147152599213, 0.]
    min_speeds = [0., 0., 0., 2.0, 0.]
    
    #  apply a small safety factor to the empirical crash speeds
    max_speeds = list(np.array(max_speeds)*.8)
    min_speeds = list(np.array(min_speeds)*1.2)
    
    # print("data x", data.transform.translation.x)
    # for each point I want to set it relative to my complete run
    point = np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
    
    # print("point before ", point)
    point -= first_point

    point = rotation.apply(point)
    # print("point with rotation", point)


        # get time from point
    # format = '%Y/%m/%d/%H:%M:%S.%f'
    # time = datetime.strptime(data.time, format)

    # compute the time delay from the last measurement to the current control cycle
    time_data = data.header.stamp  # time at which the position data was obtained
    time_now = rospy.Time.now()  # time at which this line of code is being executed
    delay = (time_now - time_data).to_sec()  # delay between our latest data measurement and right now
    # print("delay: ", delay.nsecs/1e9)

    # approximate the current velocity and control cycle duration using the last few measurements
    history_length = 4
    # # FIFO list buffer
    prev_points.append((point, time_data))
    control_times.append(time_now)
    # only keep track of the last history_length points
    if len(prev_points) > history_length:
        prev_points.pop(0)
        control_times.pop(0)
    if len(prev_points) > 1:
        avg_ctrl_time = (control_times[-1] - control_times[0]).to_sec()/(len(control_times)-1)
    else:
        avg_ctrl_time = 0.02  # conservative control cycle estimate
    # compute velocity for every point after the first history_length, and get running average of the control cycle time
    if len(prev_points) >= history_length:
        distance_traveled = 0
        for i in range(len(prev_points)-1):
            # approximate distance traveled through a straight line segment
            distance_traveled += np.linalg.norm(prev_points[i][0] - prev_points[i+1][0])
        total_time = (prev_points[-1][1] - prev_points[0][1]).to_sec()
        velocity = distance_traveled/total_time
        # print("time between last 2 measurements: ", (prev_points[-1][1] - prev_points[-2][1]).to_sec())
        # print("time since last cycle", (control_times[-1] - control_times[-2]).to_sec())
        # print("average control cycle time ", avg_ctrl_time)
    else:
        velocity = 0  # default value for velocioty
    
    # approximate distance traveled since last measurement assuming constant speed
    distance_since_measurement = velocity * delay

    # find the s on spline that corresponds to last measured position
    s_measured, _ = curve.projectPoint(point)
    # handle error with split curve of calling window curve(1,1)
    if s_measured == 1.0:
        s_measured = 0.999999
    

    # SAFETY MONITOR
    
    filter_acc = False
    filter_dec = False
    #check which point segment we are currently in
    # current_segment = -1
    # for i, segment in enumerate(segments):   
    #     if is_in_segment(point, segment):
    #         current_segment = i
    #         break
    current_segment = get_segment(s_measured, s_boundaries)
    # if current_segment == -1:
        # print("No segment found for point")
    s_segment_end = s_boundaries[current_segment]
    distance_to_next = distance_along_curve(s_measured, s_segment_end, curve)
    distance_to_next -= distance_since_measurement  # adjust for control delay: where are we now?
    # assuming we are maintaining current velocity as opposed to following humans command which we would need to be able 
    # to read state of the human command 
    distance_to_next -= velocity * avg_ctrl_time  # if we let car keep going, where will we be next time step?
    v_max_next = max_speeds[(current_segment + 1) % n_segments]
    v_min_next = min_speeds[(current_segment + 1) % n_segments]


    # if we advance into future segments, update distance and consider all velocity constraints
    while (distance_to_next < 0):
        if velocity > v_max_next:
            filter_dec = True
        elif velocity < v_min_next:
            filter_acc = True
        current_segment = (current_segment + 1) % n_segments  # advance to next segment
        s_segment_start = s_segment_end
        s_segment_end = s_boundaries[current_segment]
        distance_to_next += distance_along_curve(s_segment_start, s_segment_end, curve)  # distance to the next segment's end
        next_segment = (current_segment + 1) % n_segments  # redefine the next segment
        v_max_next = max_speeds[next_segment]
        v_min_next = min_speeds[next_segment]
    
    # Safety monitor logic of the filter: need to accelerate/need to decelerate     
    if velocity > v_max_next:
        acc = max_decc
        velocity_at_boundary = math.sqrt(max(0, velocity**2 + (2*acc*distance_to_next)))
        if velocity_at_boundary > v_max_next:
            filter_dec = True
    elif velocity < v_min_next:
        acc = max_acc
        velocity_at_boundary = math.sqrt(max(0, velocity**2 + (2*acc*distance_to_next)))
        if velocity_at_boundary < v_min_next:
            filter_acc = True

    # if current_segment !=4:
    #     velocity_limit = safe_ranges[current_segment + 1]
    # else:
    #     velocity_limit = safe_ranges[0]
    # if current_segment ==2:
    #     acc = max_acc 
    # else:
    #     acc = max_decc

    # for next time step 0.1
    
    # distance_to_next_segment-= 0.1*velocity
    # velocity_at_boundary = math.sqrt(max(0, velocity**2 + (2*acc*distance_to_next_segment)))


    # SAFETY INTERVENTION
    
    # Safety intervention smooth-type logic
    #tune constant 
    # kp = 1
    # # kp(vmax - vcurr)
    # control_input = kp * (max_velocity_for_segment - velocity)

    # now i have slope and intercept 
    # copy and paste the slope and itnerfcept from experiment 
    # slope =  0.0211393133757082
    # intercept =  0.513633764310587
    # # #setting the angle to the safe range velocity
    # set_angle = (velocity_limit - intercept) / slope 
    # angle = int(set_angle)

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
    # print(current_segment)
    if filter_dec:
            angle= 0
            print("Override control with max deccel")
            ser.write(pack('3B', 255, angle, 255))
            time.sleep(.1)
    elif filter_acc:
            angle=90
            print("Override control with max accel")
            ser.write(pack('3B', 255, angle, 255))
            time.sleep(.1)


    # print("points", s_point)
            angle of vmin or vmax [of next segment]

    
    # Safety intervention switching-typ logic (either max accel or min decel)
    try:
        if filter_dec:
            angle= 0
            print("Override control with max deccel")
            ser.write(pack('3B', 255, angle, 255))
            time.sleep(.1)
        elif filter_acc:
            angle=90
            print("Override control with max accel")
            ser.write(pack('3B', 255, angle, 255))
            time.sleep(.1)

    # try:

    #     # print("limit", velocity_limit)
    #     # print("boundary", velocity_at_boundary)

        
    #     if velocity_at_boundary > velocity_limit and current_segment != 2:
    #         angle= 0
    #         print("Override control with max deccel")
            
    #         ser.write(pack('3B', 255, angle, 255))
    #         time.sleep(.1)
    #     if velocity_at_boundary < velocity_limit and current_segment == 2:
    #         angle=90
    #         print("Override control with max accel")
    #         ser.write(pack('3B', 255, angle, 255))
    #         time.sleep(.1)
        

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
        ser=serial.Serial(baudrate='9600', timeout=.5, port='/dev/tty.usbmodem1101', write_timeout=0.2)
    except:
        print('Port open error')
    listener(ser)
