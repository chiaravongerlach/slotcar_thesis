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

# line_1 = np.array([[0.5594, 1.1028], [0.4208, 0.4009]]) 
# # Turn 1
# turn_1 = np.array([[line_1[0, 0], line_1[1, 1]], [0.0234, 0.0405]])
# # Line 2
# line_2 = np.array([[0.1993, 0.6942],[turn_1[1,0], line_1[1, 1]]])
# # Loop
# loop = np.array([[line_2[0,0],line_1[0,1]],[-0.0645, line_2[0,1]]])
# # Second Turn
# turn_2 = np.array([[line_1[0,0], 1.4731], [loop[1,0], line_1[0,1]]])

# # Set relative to starting point
# start_point = np.array([0.52252043, 1.04219351])
# line_1 -= start_point
# turn_1 -= start_point
# line_2 -= start_point
# loop -= start_point
# turn_2 -= start_point

# # Use this to iterature through the segments later on when you wanna check what point crashed 
# segments = [line_1, turn_1, line_2, loop, turn_2]

# file = pd.read_csv('../first_run/complete3.csv')

# # Create numpy array of each position in complete csv file 
# complete_positions = file[['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
# complete_array = complete_positions.to_numpy()
# start_point = np.copy(complete_array[0, 0:2])
# complete_array -= complete_array[0]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# Scatter plot using 'x', 'y', 'z' columns, 'c' to color code by velocity 
# sc = ax.scatter(complete_array[:,0], complete_array[:,1],complete_array[:,2], c=file['v_magnitude'], cmap='viridis')

# Create a colorbar to show the time values
# cbar = plt.colorbar(sc)
# cbar.set_label('Velocity')
# Set labels for axes
ax.set_xlabel('X position')
ax.set_ylabel('Y position')
ax.set_zlabel('Z position')
# Set title
ax.set_title('Robot Position Over Time')
# # Draw the rectangle on the xy plane
# draw_rectangle(ax, line_1, z=0)
# # Draw the rectangle on the xy plane
# draw_rectangle(ax, line_2, z=0)
# # Draw the rectangle on the xy plane
# draw_rectangle(ax, turn_1, z=0)
# # Draw the rectangle on the xy plane
# draw_rectangle(ax, turn_2, z=0)
# # Draw the rectangle on the xy plane
# draw_rectangle(ax, loop, z=0)



path = '../angle'
file1 = '../angle/120.csv'

df = pd.read_csv(file1)
complete_positions = df[['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
complete_array = complete_positions.to_numpy()
ax.scatter(complete_array[:,0], complete_array[:,1],complete_array[:,2])





file2 = '../angle/090.csv'
# csv_files = glob.glob(path + "/*.csv")
df = pd.read_csv(file2)
complete_positions = df[['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
complete_array = complete_positions.to_numpy()
ax.scatter(complete_array[:,0]+.1, complete_array[:,1],complete_array[:,2])




plt.show()