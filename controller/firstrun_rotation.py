import csv 
import pickle
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

# Create numpy array of each position in complete csv file 
complete_positions = file[['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
complete_array = complete_positions.to_numpy()
complete_array -= complete_array[0]
start_point = np.copy(complete_array[0, 0:2])

for point in complete_array:
    # find the distance between the point you are at and the beginning
    distance_line = np.linalg.norm(point - complete_array[0])
    #once this distance is more than 0.2 (this is why it needs to use the filtered points so that it dosn't use an outlier)
    # then we make our vector and adjust the yaw 
    if distance_line > 0.5:
        vector_complete = point - complete_array[0]

## for other run 
df = pd.read_csv('../first_run/firstrun.csv')
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

# Segements 
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


# Pickle the object and save it to a file
with open('rotation_pickle.pkl', 'wb') as file:
    pickle.dump((rotation_axis, positions_reset[0], segments), file)
