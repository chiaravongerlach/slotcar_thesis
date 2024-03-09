import csv 
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

#breaking up the space into different segments that we can partition the track into
# square for line segment 1 
# convention, first point is top right and then bottom left 
line_1 = np.array([[0.5594, 1.0507], [0.4208, 0.4009]]) #first straight segment
# first turn
turn_1 = np.array([[line_1[0, 0], line_1[1, 1]], [0.0234, 0.0405]])
#line 2 
line_2 = np.array([[0.1993, 0.6942],[turn_1[1,0], line_1[1, 1]]])
#loop 
loop = np.array([[line_2[0,0],line_1[0,1]],[-0.0645, line_2[0,1]]])
#second turn 
turn_2 = np.array([[line_1[0,0], 1.3757], [loop[1,0], line_2[0,1]]])





