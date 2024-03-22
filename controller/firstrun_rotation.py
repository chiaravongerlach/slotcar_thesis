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


# when you first place the table under the vicon run this script to get the rotation matrix
# 

file = pd.read_csv('../first_run/complete3.csv')

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

time_complete = (file['datetime'] - file['datetime'][0]).dt.total_seconds().to_numpy()

# # Make each point relative to the starting position 
# # Set origin 
complete_array -= complete_array[0]

# Numpy of the time relative to starting time
complete_time = (file['datetime'] - file['datetime'][0]).dt.total_seconds().to_numpy()

# get vector of complete run

for i, point in enumerate(complete_array):
        # find the distance between the point you are at and the beginning
        distance_line = np.linalg.norm(point - complete_array[0])
        #once this distance is more than 0.2 (this is why it needs to use the filtered points so that it dosn't use an outlier)
        # then we make our vector and adjust the yaw 
        if distance_line > 0.5:
            vector_complete = point - complete_array[0]

## for other run 
df = pd.read_csv('../first_run/first.csv')
transformations = df[['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
# Convert the pandas DataFrame to a NumPy array
positions = transformations.to_numpy()
positions_reset = positions - positions[0]

for point in positions_reset:
    # find the distance between the point you are at and the beginning
    distance_line = np.linalg.norm(point - positions_reset[0])
    #once this distance is more than 0.2 (this is why it needs to use the filtered points so that it dosn't use an outlier)
    # then we make our vector and adjust the yaw 
    if distance_line > 0.5:
        vector_csv = point - positions_reset[0]
        break
    # rotate each point 
    # positions_reset = rotation.apply(positions_reset)
    # normalize vector and vector_csv 
    vector_norm = vector_complete / np.linalg.norm(vector_complete)
    vector_norm_csv = vector_csv /np.linalg.norm(vector_csv)
    # print("vector norm", vector_norm, vector_norm_csv)
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
