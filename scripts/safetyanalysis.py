import csv 
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import LSQUnivariateSpline
from scipy.optimize import minimize_scalar, basinhopping, fminbound, differential_evolution
from scipy.spatial.transform import Rotation as R
import glob 
import math
from pyspline import Curve 
import pickle

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

complete_array = complete_array[0:710]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Scatter plot using 'x', 'y', 'z' columns, 'c' to color code by velocity 
sc = ax.scatter(complete_array[:,0], complete_array[:,1],complete_array[:,2])


# Scatter plot using 'x', 'y', 'z' columns, 'c' to color code by velocity 
# sc = ax.scatter(complete_array[:,0], complete_array[:,1],complete_array[:,2], c=file['v_magnitude'], cmap='viridis')

# Numpy array that has x and y of first poiint 
start_point = np.copy(complete_array[0, 0:2])
start_point_3d = np.copy(complete_array[0])

time_complete = (file['datetime'] - file['datetime'][0]).dt.total_seconds().to_numpy()

# # Make each point relative to the starting position 
# # Set origin 
complete_array -= complete_array[0]
final_crashing_velocities = []
# # Normalize rotation relative to a yaw of complete array 
for i, point in enumerate(complete_array):
        # find the distance between the point you are at and the beginning
        distance_line = np.linalg.norm(point - complete_array[0])
        #once this distance is more than 0.2 (this is why it needs to use the filtered points so that it dosn't use an outlier)
        # then we make our vector and adjust the yaw 
        if distance_line > 0.5:
            vector_complete = point - complete_array[0]

            distance_traveled = 0
            for r in range(i-10,i-4):
                distance_traveled += np.linalg.norm(complete_array[i] - complete_array[i+1])
            total_time = time_complete[i-4] - time_complete[i-10]
            velocity = distance_traveled/total_time
            final_crashing_velocities.append(velocity)
            distance_traveled=0
            total_time =0

            break



# Numpy of the time relative to starting time
complete_time = (file['datetime'] - file['datetime'][0]).dt.total_seconds().to_numpy()[0:710]

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




#********** LSQUnivariate Spline **********
# create 3 splines, one for x,y,z, all dependent on time 
knots = np.linspace(t_bounds[0], t_bounds[1], 100)[1:-1]
# x spline 
xspline = LSQUnivariateSpline(complete_time, complete_array[:,0], knots)
# y spline 
yspline = LSQUnivariateSpline(complete_time, complete_array[:,1], knots)
# z spline 
zspline = LSQUnivariateSpline(complete_time, complete_array[:,2], knots)

#********** Curve Spline **********
# Create Curve curve spline of the complete array  

curve_time = np.arange(complete_time[0], complete_time[-1], 0.001)
curve = Curve(x=xspline(curve_time), y=yspline(curve_time), z=zspline(curve_time), k=4)

#********** Returns minimum distance from point to complete spline   **********
# call min_distance_to_spline with the point on my multiple runs csv file that i want to find the shortest distance to my spline 
def min_distance_to_spline(point):
    s, distance_array = curve.projectPoint(point)
    distance = np.linalg.norm(distance_array)
    closest_point = curve.getValue(s)
    return distance, closest_point




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
print("start point", start_point)

# Use this to iterature through the segments later on when you wanna check what point crashed 
segments = [line_1, turn_1, line_2, loop, turn_2]

# creates a nested list where each element is representative of one segment 
crash_vel = [[] for _ in range(len(segments))]

#********** Plotting  **********
# Create a new matplotlib figure and 3D axis

# Create a colorbar to show the time values
# cbar = plt.colorbar(sc)
# cbar.set_label('Velocity')
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
#compare
# ax.plot(xspline(plot_time), yspline(plot_time), zspline(plot_time))
#Plot Curve Spline
s_values = np.linspace(0,1, 1000)
curve_points = np.array([curve.getValue(s) for s in s_values])
# #evaluate spline at every plot time 
ax.plot(curve_points[:, 0], curve_points[:,1], curve_points[:,2])

#********** Returns True if point is in segment  **********
def is_in_segment(point, segment):
    if segment[1,0] <= point[0] and point[0] <= segment[0,0] and segment[1,1] <= point[1] and point[1] <= segment[0,1]:
        return True 
    else: 
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

last_segment =0
boundary_s = []
print(len(complete_array))


# test_point = np.array([0.0057,0.85,0.033])
# test_point-= start_point_3d
# ax.plot(test_point[0], test_point[1], test_point[2], 'go', markersize = 15)
# current_segment = -1
# for i, segment in enumerate(segments):   
#     if is_in_segment(test_point, segment):
#         current_segment = i
#         break
# print(current_segment)


# iterate through points 
for pt in complete_array:
    current_segment = -1
    for i, segment in enumerate(segments):   
        if is_in_segment(pt, segment):
            current_segment = i
            break
    if current_segment != last_segment:
        s_current, _ = curve.projectPoint(pt)
        boundary_s.append(s_current.item())
        last_segment = current_segment
boundary_s[3] -= 0.02
print(boundary_s)

# # plotting the boundaries 
# for s in boundary_s:
#     point1 = curve(s)
#     ax.plot(point1[0], point1[1], point1[2], 'ko', markersize = 15)

# pickle the spline 

# Pickle the object and save it to a file
with open('spline_pickle.pkl', 'wb') as file:
    pickle.dump(curve, file)


    
















# #********** Iterate Through Runs  **********
# # To get all csv files in folder "processing"
# path = '../march 8 recordings'
# csv_files = glob.glob(path + "/*.csv")


# for f in  csv_files:
#     df = pd.read_csv(f)
#     # convert position, time to numpy 
#     positions, time = csv_to_numpy_array_pos(df)
#     time_csv = (df['datetime'] - df['datetime'][0]).dt.total_seconds().to_numpy()

#     positions_reset = positions - positions[0]
    
#     # ax.scatter(positions_reset[:,0], positions_reset[:,1],positions_reset[:,2])

    

#     filtered_points = []
#     filtered_time =[]
    

#     ignore_filtered_points = False 
#     sure_on_track = True
#     unsure_points = []
#     unsure_points_times = []
#     for s in range(len(positions_reset)):

#         if not sure_on_track and len(unsure_points) < 2:
#             unsure_points.append(positions_reset[s])
#             unsure_points_times.append(time_csv[s])
            
#             continue

#         # print("new point")
#         if s < 2:
#             filtered_points.append(positions_reset[s])
#             filtered_time.append(time_csv[s])
#             continue
#         if ignore_filtered_points:
            
#             outlier_tospline_distance, closest_point = min_distance_to_spline(positions_reset[s])
#             if (outlier_tospline_distance < .15):
#                 filtered_points.append(positions_reset[s])
#                 filtered_time.append(time_csv[s])
#                 ignore_filtered_points = False
#                 continue
        
    
#         if sure_on_track:
#             vector_between_points = filtered_points[-1] - filtered_points[-2]
#             extra_pt = vector_between_points + filtered_points[-1]
#             distance_to_extra =np.linalg.norm(positions_reset[s]-extra_pt)
#             distance_of_vector = np.linalg.norm(vector_between_points)
#             # print("distance of extra, vector", distance_to_extra, distance_of_vector)
#         else:
#             vector_between_points = unsure_points[-1] - unsure_points[-2]
#             extra_pt = vector_between_points + unsure_points[-1]
#             distance_to_extra =np.linalg.norm(positions_reset[s]-extra_pt)
#             distance_of_vector = np.linalg.norm(vector_between_points)

#         if (distance_to_extra < 0.5*distance_of_vector+0.01):
#             if sure_on_track:
#                 filtered_points.append(positions_reset[s])
#                 filtered_time.append(time_csv[s])
#             else:
#                 unsure_points.append(positions_reset[s])
#                 unsure_points_times.append(time_csv[s])
#                 if len(unsure_points) == 4:
#                     sure_on_track = True
#                     filtered_points.extend(unsure_points)
#                     filtered_time.extend(unsure_points_times)
#                     unsure_points = []
#                     unsure_points_times = []
#         else:
#             if not sure_on_track:
#                 unsure_points = []
#                 unsure_points_times = []
                

#             outlier_tospline_distance, closest_point = min_distance_to_spline(positions_reset[s])
            
#             if (outlier_tospline_distance < .05):
#                 filtered_points.append(positions_reset[s])
#                 filtered_time.append(time_csv[s])
#                 ignore_filtered_points = True
#             else:
#                 # ax.plot(positions_reset[s][0], positions_reset[s][1], positions_reset[s][2], 'go', markersize = 15)
#                 sure_on_track = False
#                 unsure_points.append(positions_reset[s])
#                 unsure_points_times.append(time_csv[s])


#     positions_reset = np.array(filtered_points)
#     time_csv = np.array(filtered_time)
    







#     for point in positions_reset:
#         distance_line = np.linalg.norm(point - positions_reset[0])
      
#         if distance_line > 0.5:
#             vector_csv = point - positions_reset[0]
#             break

#     vector_norm = vector_complete / np.linalg.norm(vector_complete)
#     vector_norm_csv = vector_csv /np.linalg.norm(vector_csv)
   
#     axis_rot = np.cross(vector_norm, vector_norm_csv)

#     axis_rot_norm = axis_rot / np.linalg.norm(axis_rot)

#     angle = np.arccos(np.dot(vector_norm, vector_norm_csv))
#     angle_degrees = angle* 180 / math.pi
 
#     rotation_vector = -angle * axis_rot_norm
#     rotation_axis = R.from_rotvec(rotation_vector)
#     positions_reset = rotation_axis.apply(positions_reset)


plt.show()