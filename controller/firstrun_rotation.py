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
    ax.plot(-0.00634452,  0.00076581,  0.00148233, 'ro', markersize=10)

# when you first place the table under the vicon run this script to get the rotation matrix
# 

file = pd.read_csv('../first_run/complete3.csv')

# Create numpy array of each position in complete csv file 
complete_positions = file[['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
complete_array = complete_positions.to_numpy()
start_point = np.copy(complete_array[0, 0:2])
complete_array -= complete_array[0]


for point in complete_array:
    # find the distance between the point you are at and the beginning
    distance_line = np.linalg.norm(point - complete_array[0])
    #once this distance is more than 0.2 (this is why it needs to use the filtered points so that it dosn't use an outlier)
    # then we make our vector and adjust the yaw 
    if distance_line > 0.5:
        vector_complete = point - complete_array[0]
        break

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
print(vector_norm, vector_norm_csv)

angle_degrees = angle* 180 / math.pi

print(angle_degrees)
#rotation vector
#length of vector is the angle of rotation 
#direction of vector is the axis that we rotate by 
rotation_vector = -angle * axis_rot_norm
rotation_axis = R.from_rotvec(rotation_vector)
#change 
positions_reset = rotation_axis.apply(positions_reset)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(positions_reset[:,0], positions_reset[:,1],positions_reset[:,2])

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
    pickle.dump((rotation_axis, positions[0], segments), file)



ax.scatter(complete_array[:,0], complete_array[:,1],complete_array[:,2])
# Scatter plot using 'x', 'y', 'z' columns, 'c' to color code by velocity 
# sc = ax.scatter(complete_array[:,0], complete_array[:,1],complete_array[:,2], c=file['v_magnitude'], cmap='viridis')
# ax.scatter()
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

plt.show()