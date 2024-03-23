import glob 
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import stats





#encode angle in the file name 
#********** Iterate Through Runs  **********
# To get all csv files in folder "processing"
path = '../angle'
csv_files = glob.glob(path + "/*.csv")

angles = []
velocities = []


for f in  csv_files:
    df = pd.read_csv(f)
    #get angle from file name 
    #last part of path should be number of angle.csv
    # angle 2 would be 002.csv
    angle = int(f[-7:-4])

    transformations = df[['.transform.translation.x', '.transform.translation.y', '.transform.translation.z']]
    # Convert the pandas DataFrame to a NumPy array
    positions = transformations.to_numpy()
    df['datetime'] = pd.to_datetime(df['time'], format = '%Y/%m/%d/%H:%M:%S.%f')
    #create new column that calculates the time difference from every consecutive line 
    time_csv = (df['datetime'] - df['datetime'][0]).dt.total_seconds().to_numpy()
    #convert time to numpy 
    #time = csv_to_numpy_array_time(df)
    #set relative to origin start point , now every run starts at 0,0,0
    positions_reset = positions - positions[0]


    for i, point in enumerate(positions_reset):
        # find the distance between the point you are at and the beginning
        distance_line = np.linalg.norm(point - positions_reset[0])
        #once this distance is more than 0.2 (this is why it needs to use the filtered points so that it dosn't use an outlier)
        # then we make our vector and adjust the yaw 
        distance_traveled = 0
        if distance_line > 0.3:
            for r in range(i-5,i):
                distance_traveled += np.linalg.norm(positions_reset[r] - positions_reset[r+1])
            total_time = time_csv[i] - time_csv[i-5]
            velocity = distance_traveled/total_time
            break
    angles.append(angle)
    velocities.append(velocity)

#for each file it appends the angle and its corresponding velocity 
    
# plot relationships 

slope, intercept, r_value, p_value, std_err = stats.linregress(angles, velocities)
print("slope: ", slope)
print("intercept: ", intercept)
line = [slope * x + intercept for x in angles]
# Create the plot
plt.figure(figsize=(10, 6))
plt.scatter(angles, velocities, color='blue', label='Data points')
plt.plot(angles, line, color='red', label='Line of best fit: y={:.2f}x+{:.2f}'.format(slope, intercept))

# Adding labels and legend
plt.xlabel('Angle')
plt.ylabel('Velocity')
plt.title('Linear Relationship between Angle and Velocity')
plt.legend()

# Show plot
plt.show()
    

