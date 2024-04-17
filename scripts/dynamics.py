import matplotlib.pyplot as plt

# Sample data
s = [1, 2, 3, 4, 5]
v_1 = [2.1043911825424666, 1.8191871053204132, 2.4426514688135, 3., 2.1777409348338157]
v_2 = [0., 0., 0., 2.0, 0.]

# Determine boundaries for shading
lower_boundary = 0  # Change as needed
upper_boundary = 4  # Change as needed

plt.figure(figsize=(10, 5))

# Plotting the stepped lines
plt.step(s, v_1, label='max speeds', where='mid', linewidth=2)
plt.step(s, v_2, label='min speeds', where='mid', linewidth=2)

# Filling the outside regions first
plt.fill_between(s, lower_boundary, v_1, step='mid', color='red', alpha=0.3, label='Failure Set Below Max Speeds')
plt.fill_between(s, v_2, upper_boundary, step='mid', color='red', alpha=0.3, label='Failure Set Above Min Speeds')

# Then filling between the stepped lines
plt.fill_between(s, v_1, v_2, step='mid', color='blue', alpha=0.5, label=r'$\Omega$ - Safe Set')

plt.xticks([1, 2, 3, 4, 5], ['' for _ in range(len(s))])
# Adding labels and title
plt.xlabel('s')
plt.ylabel('v')
plt.title('Safe Set / Failure Set')
plt.legend()

# Show the plot
plt.show()