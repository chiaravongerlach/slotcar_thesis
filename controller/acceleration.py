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
import json
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

# import velocitites 

with open('list_vel_accel', 'r') as file:
    velocity_and_times = json.load(file)

velocities = velocity_and_times[0]
times = velocity_and_times[1]

print(len(velocities), len(times))

# get rid of the starting points since the velocity isn't stable 
plt.plot(times[30:40], velocities[30:40])

#to get slope 

slope, _ = np.polyfit(times[30:40], velocities[30:40], 1)
print("max accel", slope)
plt.show()
