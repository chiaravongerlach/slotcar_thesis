import csv 
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import LSQUnivariateSpline
from scipy.optimize import minimize_scalar, basinhopping
from scipy.spatial.transform import Rotation as R
import glob 
import math
from pyspline import Curve 

#********** Check Sampling of data collection  **********
def check_sampling(loop, file):
    """ Take in the matplotlib 3D axis: ax,
    two corners [[x, y],[x, y]]
    draw it on the plane z=0
    """
    distances = []
    j = 0
    max_distance = 0
    for i in range(1, len(file)):
        point1 = file.loc[i - 1, ['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
        point2 = file.loc[i, ['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
        distance = np.sqrt((point2['.transform.translation.x'] - point1['.transform.translation.x'])**2 + (point2['.transform.translation.y'] - point1['.transform.translation.y'])**2 + (point2['.transform.translation.z'] - point1['.transform.translation.z'])**2)
        if point1['.transform.translation.x'] < loop[0,0] and point1['.transform.translation.x'] > loop[1,0] and point1['.transform.translation.y'] < loop[0,1] and point1['.transform.translation.y'] > (loop[1,1] - .3):
            j+=1
            continue
        if distance > max_distance:
            max_distance = distance
            p1_tmp = point1
            p2_tmp = point2
        #distances.append(distance)
    # Calculate and print the maximum distance
    #max_distance = max(distances)
    print(p1_tmp['.transform.translation.x'], p1_tmp['.transform.translation.y'], p1_tmp['.transform.translation.z'])
    print(p2_tmp['.transform.translation.x'], p2_tmp['.transform.translation.y'], p2_tmp['.transform.translation.z'])
    print(f"Maximum distance: {max_distance}")
    print(j)

#RESULT : the max distance is 0.01880276944725993 which is greater than 1 cm so we are going to use a spline to interpolate the data


#********** Complete run manipulations **********

#Read in the csv file that contains a complete run
file = pd.read_csv('../rosbag_records/complete3.csv')
#Convert the <string> time into datetime and append to files object
file['datetime'] = pd.to_datetime(file['time'], format = '%Y/%m/%d/%H:%M:%S.%f')
# new column delta_t calculates the difference between each consecutive row
file['delta_t'] = file['datetime'].diff().dt.total_seconds()

# Create numpy array of each position in complete csv file 
complete_positions = file[['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
complete_array = complete_positions.to_numpy()
# print the first point in the complete_array
#print ("first point = %s " % complete_array[0] )

# Numpy array that has x and y of first poiint 
start_point = np.copy(complete_array[0, 0:2])

# # Make each point relative to the starting position 
# # Set origin 
complete_array -= complete_array[0]
# # Normalize rotation relative to a yaw of complete array 
for point in complete_array:
        # find the distance between the point you are at and the beginning
        distance_line = np.linalg.norm(point - complete_array[0])
        #once this distance is more than 0.2 (this is why it needs to use the filtered points so that it dosn't use an outlier)
        # then we make our vector and adjust the yaw 
        if distance_line > 0.5:
            vector_complete = point - complete_array[0]
            x = vector_complete[0]
            y = vector_complete[1]
            # yaw = ata
            yaw = math.atan2(y, x)
            break

# # get rotation from csv
# x = file['.transform.rotation.x'][0]
# y = file['.transform.rotation.y'][0]
# z = file['.transform.rotation.z'][0]
# w = file['.transform.rotation.w'][0]
# #get yaw from quaternions in radians 
# yaw=math.atan2(2 * (w*z+x*y), w*w+x*x-y*y-z*z)
# yaw = yaw*180/math.pi
# rotate_by = 90-yaw
# print("yaw", yaw)
# # make rotation matrix 
# rotation = R.from_euler('z', rotate_by, degrees = True)
# #rotate each point 
# # complete_array = rotation.apply(complete_array)




# Numpy of the time relative to starting time
complete_time = (file['datetime'] - file['datetime'][0]).dt.total_seconds().to_numpy()

# Time bounds
t_bounds = (complete_time[0], complete_time[-1])
# print(start_point)


#********** calculating velocity for the complete run **********

#calculate differene in positions to compute vx, vy, vz
#divide by 1000 since the position is in milimeters
file['vx'] = file['.transform.translation.x'].diff() /1000 / file['delta_t']
file['vy'] = file['.transform.translation.y'].diff() /1000 /  file['delta_t']
file['vz'] = file['.transform.translation.z'].diff() /1000 / file['delta_t']
# calculate the magnitude of the velocities 
file['v_magnitude'] = np.sqrt(file['vx']**2 + file['vy']**2 + file['vz']**2)
#see the first five rows
# print(file.head())

#********** Curve Spline **********
# Create Curve curve spline of the complete array  
curve = Curve(X = complete_array, k =4)


#********** LSQUnivariate Spline **********
# create 3 splines, one for x,y,z, all dependent on time 
knots = np.linspace(t_bounds[0], t_bounds[1], 100)[1:-1]
# x spline 
xspline = LSQUnivariateSpline(complete_time, complete_array[:,0], knots)
# y spline 
yspline = LSQUnivariateSpline(complete_time, complete_array[:,1], knots)
# z spline 
zspline = LSQUnivariateSpline(complete_time, complete_array[:,2], knots)

#********** Distance between point x,y,z and LSQUnivariate Spline  **********
def distancespline_function(t, x, y, z): 
    x_t = xspline(t)
    y_t = yspline(t)
    z_t = zspline(t)
    return np.sqrt((x_t-x)**2 + (y_t-y)**2 + (z_t-z)**2)

#********** Returns minimum distance from point to complete spline   **********
# call min_distance_to_spline with the point on my multiple runs csv file that i want to find the shortest distance to my spline 
def min_distance_to_spline(point):
    x, y, z = point

    def distance_function(t): 
        x_t = xspline(t)
        y_t = yspline(t)
        z_t = zspline(t)
        return np.sqrt((x_t-x)**2 + (y_t-y)**2 + (z_t-z)**2)
    # black box optimization

    #minimize scalar 
    # res = minimize_scalar(distance_function, bounds = t_bounds, method='bounded', options={'maxiter': 5000}, bracket = (4,5))
    # global minimizer 

    bounds = [(complete_time[0], complete_time[-1])]
    kwargs = {"method": "L-BFGS-B", "bounds": bounds}
    x_init =[0]
    res = basinhopping(distance_function, x_init, minimizer_kwargs=kwargs, niter=500, stepsize =19)

    # print("nit", res.nit)
    # print("res.x", res.x)
    # print("tbounds", t_bounds)
    # ax.plot(xspline(res.x), yspline(res.x), zspline(res.x), 'ro')

    # #testing to see that our spline works and the result is that for an arbitrary point our spline has  a very similiar point 
    # print("distance 7", distance_function(7))
    # print("distance 0", distance_function(0))
    # print("distance 3", distance_function(5))
    # print("time", complete_time[540])
    # print("point at 100", complete_array[540])
    # print("spline at time 100", xspline(complete_time[540]), yspline(complete_time[540]), zspline(complete_time[540]))
    # print("distance to 100", distance_function(complete_time[540]))
    # # res.x is the time along my spline that corresponds to the shortest distance on the spline to my point
    # print("spline at time", xspline(res.x), yspline(res.x), zspline(res.x))
    return res.fun, res.x


#********** Function to draw each segement of the track  **********
def draw_rectangle(ax, corners, z=0):
    """ Take in the matplotlib 3D axis: ax,
    two corners [[x, y],[x, y]]
    draw it on the plane z=0
    """
    # Check corners is a np array
    corners = np.array(corners)
    # Calculate the other two corners of the rectangle
    # Get first corner and label it corner1
    corner1 = corners[0]
    corner2 = corners[1]
    # Create the two other corners of the rectangle
    corner3 = np.array([corner1[0], corner2[1]])
    corner4 = np.array([corner2[0], corner1[1]])

    # Combine corner 1,2,3,4 and plot the rectangle_corners
    rectangle_corners = np.array([corner1, corner3, corner2, corner4, corner1])
    ax.plot(rectangle_corners[:, 0], rectangle_corners[:, 1], zs=z, zdir='z', marker='o')

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
line_1 -= start_point
turn_1 -= start_point
line_2 -= start_point
loop -= start_point
turn_2 -= start_point

# Use this to iterature through the segments later on when you wanna check what point crashed 
segments = [line_1, turn_1, line_2, loop, turn_2]

# creates a nested list where each element is representative of one segment 
crash_vel = [[] for _ in range(len(segments))]

#********** Plotting  **********
# Create a new matplotlib figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Scatter plot using 'x', 'y', 'z' columns, 'c' to color code by velocity 
sc = ax.scatter(complete_array[:,0], complete_array[:,1],complete_array[:,2], c=file['v_magnitude'], cmap='viridis')
# Create a colorbar to show the time values
cbar = plt.colorbar(sc)
cbar.set_label('Velocity')
# Set labels for axes
ax.set_xlabel('X position')
ax.set_ylabel('Y position')
ax.set_zlabel('Z position')
# Set title
ax.set_title('Robot Position Over Time')
# Draw the rectangle on the xy plane
draw_rectangle(ax, line_1, z=0)
# Draw the rectangle on the xy plane
draw_rectangle(ax, line_2, z=0)
# Draw the rectangle on the xy plane
draw_rectangle(ax, turn_1, z=0)
# Draw the rectangle on the xy plane
draw_rectangle(ax, turn_2, z=0)
# Draw the rectangle on the xy plane
draw_rectangle(ax, loop, z=0)
## plot the spline 
#make time vector 
plot_time = np.arange(t_bounds[0], t_bounds[1], 0.01)
#Plot LSQUnivariate Spline
ax.plot(xspline(plot_time), yspline(plot_time), zspline(plot_time))
#Plot Curve Spline
# s_values = np.linspace(0,1, 1000)
# curve_points = np.array([curve.getValue(s) for s in s_values])
# #evaluate spline at every plot time 
# ax.plot(curve_points[:, 0], curve_points[:,1], curve_points[:,2])

#********** Returns True if point is in segment  **********
def is_in_segment(point, segment):
    if segment[1,0] <= point[0] and point[0] <= segment[0,0] and segment[1,1] <= point[1] and point[1] <= segment[0,1]:
        return True 
    else: 
        print("hgbrs")
        return False
    

#********** Turn DF file into numpy array for positions, delta_t  **********
def csv_to_numpy_array_pos(df):
    # Select the columns corresponding to x, y, z transformations
    transformations = df[['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
    # Convert the pandas DataFrame to a NumPy array
    transformations_array = transformations.to_numpy()
    df['datetime'] = pd.to_datetime(df['time'], format = '%Y/%m/%d/%H:%M:%S.%f')
    #create new column that calculates the time difference from every consecutive line 
    time_1 = df['datetime'].diff().dt.total_seconds()
    delta_t = time_1.to_numpy()
    return transformations_array, delta_t

def csv_to_numpy_array_time(df):
    df['datetime'] = pd.to_datetime(df['time'], format = '%Y/%m/%d/%H:%M:%S.%f')
    time_array = (df['datetime'] - df['datetime'][0]).astype('timedelta64[ns]').astype('float64') / 1e9
    # time_array = time_deltas_ns / 1e9
    return time_array.to_numpy()


#********** Iterate Through Runs  **********
# To get all csv files in folder "processing"
path = '../processing'
csv_files = glob.glob(path + "/*.csv")



for f in  csv_files:
    df = pd.read_csv(f)
    # convert position, time to numpy 
    positions, time = csv_to_numpy_array_pos(df)
    time_csv = (df['datetime'] - df['datetime'][0]).dt.total_seconds().to_numpy()
    #convert time to numpy 
    #time = csv_to_numpy_array_time(df)
    #set relative to origin start point , now every run starts at 0,0,0
    positions_reset = positions - positions[0]


    #****** old way of doing relative
    # print("shape before", positions_reset.shape)
    # #rotate relative to complete run 
    # x = df['.transform.rotation.x'][0]
    # y = df['.transform.rotation.y'][0]
    # z = df['.transform.rotation.z'][0]
    # w = df['.transform.rotation.w'][0]
    # #get yaw from quaternions in radians 
    # yaw_new=math.atan2(2 * (w*z+x*y), w*w+x*x-y*y-z*z)
    # yaw_new = yaw_new*180/math.pi
    # print("yaw", yaw_new)
    # # make rotation matrix 
    # rotation = R.from_euler('z', yaw - yaw_new, degrees = True)
    # #rotate each point 
    # positions_reset = rotation.apply(positions_reset)
    # print("shape after", positions_reset.shape)

 

    #FILTER

    #to deal with the outliers that can falsely say we crashed before we did we check the distance for each point to the point before 
    # andw the point after and if both distances are greater than 3 centimeters then we skip over this point so that we dont 
    # count that as the crash 

    #Duy : outliers

    filtered_points = []
    filtered_time =[]
    print("length before", len(positions_reset))

    ignore_filtered_points = False 
    for s in range(len(positions_reset)):
        print("new point")

        if s < 2:
            filtered_points.append(positions_reset[s])
            filtered_time.append(time_csv[s])
            continue
        
        if ignore_filtered_points:
            print("call to mindist")
            outlier_tospline_distance, time_outlier = min_distance_to_spline(positions_reset[s])
            if (outlier_tospline_distance < .15):
                filtered_points.append(positions_reset[s])
                filtered_time.append(time_csv[s])
                ignore_filtered_points = False
                continue

        # vector =positions_reset[s-1] - positions_reset[s-2]
        vector_between_points = filtered_points[-1] - filtered_points[-2]
        extra_pt = vector_between_points + positions_reset[s-1]
        distance_to_extra =np.linalg.norm(positions_reset[s]-extra_pt)
        distance_of_vector = np.linalg.norm(vector_between_points)
        print("distance of extra, vector", distance_to_extra, distance_of_vector)
        if (distance_to_extra < 0.5*distance_of_vector+0.01):
            filtered_points.append(positions_reset[s])
            filtered_time.append(time_csv[s])
            continue
        print("call2 to mindist")
        outlier_tospline_distance, time_outlier = min_distance_to_spline(positions_reset[s])
        if (outlier_tospline_distance < .15):
            filtered_points.append(positions_reset[s])
            filtered_time.append(time_csv[s])
            ignore_filtered_points = True

    # filtered points 
    positions_reset = np.array(filtered_points)
    updated_time = np.array(filtered_time)
    print("len after", len(positions_reset))  









        
        
        # #calculate distance to next point
        # next_outlier = True
        # prev_outlier = True
        # if s < len(positions_reset) - 1:
        #     next_dist = np.linalg.norm(positions_reset[s] - positions_reset[s+1])
        #     if next_dist < 0.03:
        #         next_outlier = False
        # if s > 0:
        #     prev_dist = np.linalg.norm(positions_reset[s] - positions_reset[s-1])
        #     if prev_dist < 0.03:
        #         prev_outlier = False
        
        # if not next_outlier or not prev_outlier:
        #     filtered_points.append(positions_reset[s])
      

    # find a straight segment in the path of the first line segment and make a line between the two points. then find the yaw of that 
    # and make the starting position of that yaw the same as the starting position of the complete run yaw 
    for point in positions_reset:
        # find the distance between the point you are at and the beginning
        distance_line = np.linalg.norm(point - positions_reset[0])
        #once this distance is more than 0.2 (this is why it needs to use the filtered points so that it dosn't use an outlier)
        # then we make our vector and adjust the yaw 
        if distance_line > 0.5:
            vector_csv = point - positions_reset[0]
            x = vector_csv[0]
            y = vector_csv[1]
            # yaw = ata
            # yaw_new = math.atan2(y, x)
            # make rotation matrix 
            # rotation = R.from_euler('z', yaw - yaw_new)
            break
    # rotate each point 
    # positions_reset = rotation.apply(positions_reset)
    # normalize vector and vector_csv 
    vector_norm = vector_complete / np.linalg.norm(vector_complete)
    vector_norm_csv = vector_csv /np.linalg.norm(vector_csv)
    print("vector norm", vector_norm, vector_norm_csv)
    #cross product of the two vectors to get the axis of rotation (that is perpendicular to both)
    axis_rot = np.cross(vector_norm, vector_norm_csv)
    # create rotation matrix 
    #normalize axis of rotation
    axis_rot_norm = axis_rot / np.linalg.norm(axis_rot)
    # angle to rotate by 
    angle = np.arccos(np.dot(vector_norm, vector_norm_csv))
    angle_degrees = angle* 180 / math.pi
    #rotation vector
    #length of vector is the angle of rotation 
    #direction of vector is the axis that we rotate by 
    rotation_vector = -angle * axis_rot_norm
    rotation_axis = R.from_rotvec(rotation_vector)
    positions_reset = rotation_axis.apply(positions_reset)
    print("rotation vcector", rotation_vector)
    print("angle", angle_degrees)
    



    



    # go through every point and see how far you are from the spline 

    for i in range(len(positions_reset)):
        #print("shape of point", positions_reset[i].shape)
        

    
        # #curve
        # closest_point, distance_array = curve.projectPoint(positions_reset[i])
        # distance = np.linalg.norm(distance_array)
        # print(" distance array", distance_array)
        # ## for second point 
        # closest_point_1, distance_array_1 = curve.projectPoint(positions_reset[i+10])
        # distance_1 = np.linalg.norm(distance_array_1)
        # # for third points
        # closest_point_2, distance_array_2 = curve.projectPoint(positions_reset[i+15])
        # distance_2 = np.linalg.norm(distance_array_2)



        #lsq
        print("point we call distance function on", positions_reset[i])
        distance, time_crash = min_distance_to_spline(positions_reset[i])
        print("distance of that point", distance)
      
        # print("**********************")
        
        # print("distance to spline =  %s" % distance)
        # print("spline x at that time  =  %s" % xspline(time[i]))
        # print("spline y at that time  =  %s" % yspline(time[i]))
        # print("spline z at that time  =  %s" % zspline(time[i]))
        #check if distance is over threshold, 5 cm
        if distance > 0.1: # and distance_1 > 0.01 and distance_2 > 0.01 :
            print("shape of pos", positions_reset[i])
            ax.plot(xspline(time_crash), yspline(time_crash), zspline(time_crash), 'ro', markersize=10)
            ax.plot(positions_reset[i][0], positions_reset[i][1], positions_reset[i][2], 'go', markersize = 10)
            print("distance to spline", distance)
    


            # print("inittial  point %s " % positions_reset[i])
            # print("second point %s " % positions_reset[i+10])
            # print("thirs point %s " % positions_reset[i+15])

            #to cover if it crashes before
            if (i < 4): 
                continue
            #calculate the velocity 
            time_total = 0
            distance_traveled = 0
            pos = 0
            for r in range(i-11,i-9):
                time_total += updated_time[r]
                distance_traveled += np.linalg.norm(positions_reset[r] - positions_reset[r+1])
                print("test")
                
            #     temp = np.linalg.norm(positions_reset[r+1] - positions_reset[r]) / time[r]
            #     # print("distance", np.linalg.norm(positions_reset[r+1] - positions_reset[r]))
            #     # print("time", time[r])
            #     # print("pos", temp)
            #     pos += np.linalg.norm(positions_reset[r+1] - positions_reset[r]) / time[r]
            # velocity = pos /5
            #for one velocity 
            time_diff = updated_time[i-11] - updated_time[i-8]
            velocity = distance_traveled/ time_diff
         


            
            #get the velocity a few time steps before 

            #plot the crash 
            ax.scatter(positions_reset[:,0], positions_reset[:,1],positions_reset[:,2])




            for j, segment in enumerate(segments):
                print("crashing point =  %s" % positions_reset[i])
                print("point 10 before recorded crash", positions_reset[i-5])
                print("distance to spline", min_distance_to_spline(positions_reset[i-10]))
                
                if is_in_segment(positions_reset[i-10], segment):
                    
                    crash_vel[j].append(velocity)
                    break
            #shoudl this break be one less indented 
            break
#print crash vel 
print(crash_vel)

for zone in crash_vel:
    print(len(zone))



# find arbitrary point 
# pt = np.array([0.51592343, 0.93665992, 0.02264758])
# print("shape of arb", pt.shape)
# distance_to_arbitrary = min_distance_to_spline(pt)
# #ax.plot(pt[0], pt[1], pt[2], 'go')
# print("distance", distance_to_arbitrary)
    
    



plt.show()






# import the complete csv file 
# import the multiple runs csv file 
# take xyz points from both and place them both in 2d seperate numpy arrays (for the next csv file only do this for the multiple runs)
# do for loop that iterates through each row in numpy array of multiple run array and takes each row and subtracts it from the complete run 
# 
# complete = np.aray of complete run 
# complete - <name of multiple run array>[i]
# ^ will return np array of shape complete and from that take the magnitude 
# axis is either 0 or 1 
# magnitude = np.linalg.norm(name of array, axis = 0 or 1)
# then find the minimum. this is the smallest distance. min = np.min(magnitude) returns scalar
# check if min distance is more than threshold, if so it fell off the track 
#   so now you know for that i you fell off the track 
    # cehck what zone it is in 
    #check how fast it was going by calculating the velocity from the original multiple runs np array with x,y,z 
    # append that velocity to a nested 

    
# now each csv file that i use will append one number to crash_vel so i need a lot of csv files that i can pit this whole thing into another for 
# loop that goes through the csv files and finds the crashing velocities 
            
# for the loop when i find my crash use a point i-10 and find that velocity to append 
# once i have this big crash_vel i need to find some data processing to first get rid of outliers and maybe take the min for each one - something to make sure its safe 

# for each row in the multiple csv file I will take each point and find the closest point on the complete csv file 
# #gpt code for transforming the xyz from df file into numpy array 
            