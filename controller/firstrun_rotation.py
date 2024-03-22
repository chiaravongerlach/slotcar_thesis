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

# Pickle the object and save it to a file
with open('rotation_pickle.pkl', 'wb') as file:
    pickle.dump(rotation_axis, file)
