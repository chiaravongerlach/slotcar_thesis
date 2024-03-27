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

# import velocitites 

with open('list_vel', 'r') as file:
    velocity_and_times = json.load(file)

velocities = velocity_and_times[0]
times = velocity_and_times[1]

slope = 

print("decceleration", slope)



