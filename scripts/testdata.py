import csv 
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import LSQUnivariateSpline
from scipy.optimize import minimize_scalar
import glob 
from pyspline import Curve 



# read in the csv file 
file = pd.read_csv('../rosbag_records/complete3.csv')
# convert the time into datetime from string and append to my files object 
file['datetime'] = pd.to_datetime(file['time'], format = '%Y/%m/%d/%H:%M:%S.%f')

#create new column that calculates the time difference from every consecutive line 
file['delta_t'] = file['datetime'].diff().dt.total_seconds()

#calculate differene in positions to compute vx, vy, vz
# divide by 1000 since the position is in milimeters
file['vx'] = file['.transform.translation.x'].diff() /1000 / file['delta_t']
file['vy'] = file['.transform.translation.y'].diff() /1000 /  file['delta_t']
file['vz'] = file['.transform.translation.z'].diff() /1000 / file['delta_t']

# calculate the magnitude of the velocities 
file['v_magnitude'] = np.sqrt(file['vx']**2 + file['vy']**2 + file['vz']**2)


#see the first five rows
# print(file.head())


##### segementing 

def draw_rectangle(ax, corners, z=0):
    """
    Draw a rectangle on the xy-plane of a 3D plot given two diagonal corners.

    Parameters:
    - ax: matplotlib 3D axis.
    - corners: numpy array of shape (2, 2), where each row is an (x, y) pair representing a corner.
    - z: The z-coordinate where the rectangle is to be drawn.
    """
    # Ensure corners is a numpy array
    corners = np.array(corners)
    
    # Calculate the other two corners of the rectangle
    corner1 = corners[0]
    corner2 = corners[1]
    corner3 = np.array([corner1[0], corner2[1]])
    corner4 = np.array([corner2[0], corner1[1]])

    # Combine all corners to form the rectangle
    rectangle_corners = np.array([corner1, corner3, corner2, corner4, corner1])
    
    # Plot rectangle
    ax.plot(rectangle_corners[:, 0], rectangle_corners[:, 1], zs=z, zdir='z', marker='o')


# creating SPLINE of the complete csv file 

#preprocesing the data to create spline 

complete_positions = file[['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]

# panda to numpy 
complete_array = complete_positions.to_numpy()
#print ("first point = %s " % complete_array[0] )

#curve pyspline 
curve = Curve(X = complete_array, k =4)

# numpy array that has x and y of first poiint 
start_point = np.copy(complete_array[0, 0:2])



# make each point relative to the starting position 
# complete_array -= complete_array[0]
#complete numpy of time 
complete_time = (file['datetime'] - file['datetime'][0]).dt.total_seconds().to_numpy()

# time bounds 
t_bounds = (complete_time[0], complete_time[-1])


print(start_point)


# create 3 splines, one for x,y,z, all dependent on time 
knots = np.linspace(t_bounds[0], t_bounds[1], 100)[1:-1]

# x spline 
xspline = LSQUnivariateSpline(complete_time, complete_array[:,0], knots)
# y spline 
yspline = LSQUnivariateSpline(complete_time, complete_array[:,1], knots)
# z spline 
zspline = LSQUnivariateSpline(complete_time, complete_array[:,2], knots)



#breaking up the space into different segments that we can partition the track into
# square for line segment 1 
# convention, first point is top right and then bottom left 
line_1 = np.array([[0.5594, 1.1028], [0.4208, 0.4009]]) #first straight segment
# first turn
turn_1 = np.array([[line_1[0, 0], line_1[1, 1]], [0.0234, 0.0405]])
#line 2 
line_2 = np.array([[0.1993, 0.6942],[turn_1[1,0], line_1[1, 1]]])
#loop 
loop = np.array([[line_2[0,0],line_1[0,1]],[-0.0645, line_2[0,1]]])
#second turn 
turn_2 = np.array([[line_1[0,0], 1.4731], [loop[1,0], line_1[0,1]]])

# relative to origin start pooint 
# line_1 -= start_point
# turn_1 -= start_point
# line_2 -= start_point
# loop -= start_point
# turn_2 -= start_point

#######
# use this to iterature through the segments later on when you wanna check what point crashed 
segments = [line_1, turn_1, line_2, loop, turn_2]

# creates a nested list where each element is representative of one segment 
crash_vel = [[] for _ in range(len(segments))]


# *****************************************************

# check the euclidean distance between the complete loop track to make sure there is enough sampling of < 1 cm 

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



# Add the distances as a new column in the DataFrame, if desired
# Note: The first point will not have a preceding point to calculate distance from,
# so its distance can be considered as NaN or 0 based on your analysis needs.
# file['Distance_to_Previous'] = [np.nan] + distances

# Calculate and print the maximum distance
#max_distance = max(distances)
print(p1_tmp['.transform.translation.x'], p1_tmp['.transform.translation.y'], p1_tmp['.transform.translation.z'])
print(p2_tmp['.transform.translation.x'], p2_tmp['.transform.translation.y'], p2_tmp['.transform.translation.z'])
print(f"Maximum distance: {max_distance}")
print(j)

#RESULT : the max distance is 0.01880276944725993 which is greater than 1 cm so we are going to use a spline to interpolate the data

# *****************************************************

def distancespline_function(t, x, y, z): 
    x_t = xspline(t)
    y_t = yspline(t)
    z_t = zspline(t)
    return np.sqrt((x_t-x)**2 + (y_t-y)**2 + (z_t-z)**2)

# what object is point 
# call min_distance_to_spline with the point on my multiple runs csv file that i want to find the shortest distance to my spline 
def min_distance_to_spline(point):
    x, y, z = point

    def distance_function(t): 
        x_t = xspline(t)
        y_t = yspline(t)
        z_t = zspline(t)
        return np.sqrt((x_t-x)**2 + (y_t-y)**2 + (z_t-z)**2)
    # black box optimization
    res = minimize_scalar(distance_function, bounds = t_bounds, method='bounded', options={'maxiter': 5000})

    # res.x is the time along my spline that corresponds to the shortest distance on the spline to my point
    return distance_function(res.x)


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

#
## plot the spline 
#make time vector 
plot_time = np.arange(t_bounds[0], t_bounds[1], 0.01)

#plot lsq univariate
ax.plot(xspline(plot_time), yspline(plot_time), zspline(plot_time))

#plot curve spline
# s_values = np.linspace(0,1, 1000)
# curve_points = np.array([curve.getValue(s) for s in s_values])

# #evaluate spline at every plot time 
# ax.plot(curve_points[:, 0], curve_points[:,1], curve_points[:,2])

# Show the plot



# we can use the reectangles to check if a point is within which rectangle 
# what do i do with the corner cases?? 
def is_in_segment(point, segment):
    if segment[1,0] <= point[0] and point[0] <= segment[0,0] and segment[1,1] <= point[1] and point[1] <= segment[0,1]:
        return True 
    else: 
        print("hgbrs")
        return False
    

# ******************************************* iterate through csv files 




    

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
            






# 

#
# for each row in the multiple csv file I will take each point and find the closest point on the complete csv file 
#
# 
# 
# #gpt code for transforming the xyz from df file into numpy array 
            

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





# Usage example
# file_path = 'your_file_path_here.csv'  # Replace with your actual file path
# transformations_array = csv_to_numpy_array(file_path)
# print(transformations_array)


path = '../processing'
csv_files = glob.glob(path + "/*.csv")



for f in  csv_files:
    df = pd.read_csv(f)
    # convert position to numpy 
    positions, time = csv_to_numpy_array_pos(df)
    #convert time to numpy 
    #time = csv_to_numpy_array_time(df)
    #set relative to origin start point , now every run starts at 0,0,0
    positions_reset = positions  #- positions[0]

    #to deal with the outliers that can falsely say we crashed before we did we check the distance for each point to the point before 
    # andw the point after and if both distances are greater than 3 centimeters then we skip over this point so that we dont 
    # count that as the crash 

    filtered_points = []
    print("print shape before", positions_reset[0].shape)

    for s in range(len(positions_reset)):
        #calculate distance to next point
        next_outlier = True
        prev_outlier = True
        if s < len(positions_reset) - 1:
            next_dist = np.linalg.norm(positions_reset[s] - positions_reset[s+1])
            if next_dist < 0.03:
                next_outlier = False
        if s > 0:
            prev_dist = np.linalg.norm(positions_reset[s] - positions_reset[s-1])
            if prev_dist < 0.03:
                prev_outlier = False
        
        if not next_outlier or not prev_outlier:
            filtered_points.append(positions_reset[s])

    positions_reset = np.array(filtered_points)



    # go through every point and see how far you are from the spline 

    for i in range(len(positions_reset)):
        print("shape of point", positions_reset[i].shape)
        

        
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
        distance = min_distance_to_spline(positions_reset[i])
        # print("**********************")
        ax.plot(xspline(time[i]), yspline(time[i]), zspline(time[i]), color='green', markersize = 10)
        ax.plot(positions_reset[i][0], positions_reset[i][1], positions_reset[i][2], color='red', markersize=10)
        print("distance to spline =  %s" % distance)
        print("spline x at that time  =  %s" % xspline(time[i]))
        print("spline y at that time  =  %s" % yspline(time[i]))
        print("spline z at that time  =  %s" % zspline(time[i]))
        #check if distance is over threshold, 5 cm
        if distance > 0.03: # and distance_1 > 0.01 and distance_2 > 0.01 :
            print("distance", distance)
            # print("inittial  point %s " % positions_reset[i])
            # print("second point %s " % positions_reset[i+10])
            # print("thirs point %s " % positions_reset[i+15])

            #to cover if it crashes before
            if (i < 4): 
                continue
            #calculate the velocity 
            pos = 0
            for r in range(i-2,i+3):
                print("test")
                
                temp = np.linalg.norm(positions_reset[r+1] - positions_reset[r]) / time[r]
                print("distance", np.linalg.norm(positions_reset[r+1] - positions_reset[r]))
                print("time", time[r])
                print("pos", temp)
                pos += np.linalg.norm(positions_reset[r+1] - positions_reset[r]) / time[r]
            velocity = pos /5

            
            #get the velocity a few time steps before 

            #plot the crash 
            ax.scatter(positions_reset[:,0], positions_reset[:,1],positions_reset[:,2])




            for j, segment in enumerate(segments):
                print("crashing point =  %s" % positions_reset[i])
                print("point 10 time steps after crash  =  %s" % positions_reset[i+10])
                if is_in_segment(positions[i-1], segment):
                    
                    crash_vel[j].append(velocity)
                    break
            break
#print crash vel 
print(crash_vel)

for zone in crash_vel:
    print(len(zone))

plt.show()