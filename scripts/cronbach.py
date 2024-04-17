import csv 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import LSQUnivariateSpline
from scipy.optimize import minimize_scalar, basinhopping, fminbound, differential_evolution
from scipy.spatial.transform import Rotation as R
import glob 
import math
import pandas as pd
import numpy as np

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the data
data = pd.read_csv('data.csv')

# Define the categories
categories = {'Fluency': ['Q1', 'Q2'], 'Contribution': ['Q3', 'Q4'], 
              'Trust': ['Q5', 'Q6'], 'Capability': ['Q7', 'Q8']}

# Function to calculate Cronbach's alpha
def cronbach_alpha(df):
    df_corr = df.corr()
    N = df.shape[1]  # Number of items in the scale
    numerator = N * df_corr.sum().sum()  # Sum of all correlations
    denominator = N + (N * (N - 1) * df_corr.sum().sum())
    return numerator / denominator

questions = ['Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q6', 'Q7', 'Q8']
data_questions = data[questions]  # This DataFrame now only contains the question columns

# Now proceed with the groupby operation on this filtered DataFrame
question_averages = data.groupby('Filter type')[questions].mean()


# # Now, calculate the average scores for each category by taking the mean of the questions within each category
category_averages = {category: question_averages[questions].mean(axis=1) for category, questions in categories.items()}



# # Calculate Cronbach's alpha for each category
cronbachs_alpha = {category: cronbach_alpha(data[questions]) for category, questions in categories.items()}



# # Convert the averages and Cronbach's alpha into a DataFrame for easier handling
category_averages_df = pd.DataFrame(category_averages)
cronbachs_alpha_df = pd.DataFrame(cronbachs_alpha, index=['Cronbach Alpha']).T

print(cronbachs_alpha_df)

# # Calculate standard deviation for each question within each 'Filter type'
question_std = data.groupby('Filter type')[questions].std()

# # Calculate the standard deviation for each category
category_std = {category: question_std[questions].mean(axis=1) for category, questions in categories.items()}
category_std_df = pd.DataFrame(category_std)



# Prepare data for plotting
categories_list = list(categories.keys())
filter_types = question_averages.index.unique()
n_categories = len(categories_list)
n_filter_types = len(filter_types)

# Creating a bar chart
bar_width = 0.35  # Width of the bars
index = np.arange(n_categories)  # Category index

fig, ax = plt.subplots(figsize=(12, 6))

for i, filter_type in enumerate(filter_types):
    means = category_averages_df.loc[filter_type].values
    stds = category_std_df.loc[filter_type].values
    position = index if filter_type == 'smooth' else index + bar_width
    color = 'grey' if filter_type == 'smooth' else 'orange'  # 'b' is a default color (blue)
    ax.bar(position, means, bar_width, color=color, yerr=stds, label=filter_type.capitalize())


# Add the Cronbach's alpha to the plot
for i, (category, alpha) in enumerate(cronbachs_alpha.items()):
    position = index if filter_type == 'smooth' else index + bar_width
    ax.text(position[i], means[i] + stds[i] + 0.1, f'α = {alpha:.2f}', ha='center', va='bottom')

# Formatting the plot
ax.set_xlabel('Categories')
ax.set_ylabel('Average Likert Score')
ax.set_title('Perceptions of Smooth/Switch Safety Filter')
ax.set_xticks(index + bar_width / 2)
ax.set_xticklabels(categories_list)
ax.legend()
ax.set_ylim(1, 10)  # Likert scale ranges from 1 to 7
ax.grid(False)

plt.tight_layout()
plt.show()