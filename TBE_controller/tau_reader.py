import pandas as pd
import scipy.io
import matplotlib.pyplot as plt

# df = pd.read_csv('avg_torque_profiles.csv')

# R_avg = df['R_avg'].values
# L_avg = df['L_avg'].values
# R_std = df['R_std'].values
# L_std = df['L_std'].values

data = scipy.io.loadmat('avg_torque_profiles.mat')

avg_profiles = data['avg_profiles']  # shape: (101, 2)
std_profiles = data['std_profiles']  # shape: (101, 2)
x_axis       = data['x_axis'].flatten()  # shape: (101,)

# Split into left and right
R_avg = avg_profiles[:, 0]
L_avg = avg_profiles[:, 1]
print(R_avg)
R_std = std_profiles[:, 0]
L_std = std_profiles[:, 1]

fig, axes = plt.subplots(1, 2, figsize=(12, 5))
sides = ['Right', 'Left']
avgs  = [R_avg, L_avg]
stds  = [R_std, L_std]

for i, ax in enumerate(axes):
    ax.fill_between(x_axis, avgs[i] - stds[i], avgs[i] + stds[i],
                    alpha=0.3, color='steelblue', label='±1 SD')
    ax.plot(x_axis, avgs[i], color='steelblue', linewidth=2, label='Mean')
    ax.set_xlabel('Stance Phase (%)')
    ax.set_ylabel('Ankle Torque (Nm)')
    ax.set_title(f'{sides[i]} Ankle - Average Stance Torque Profile')
    ax.legend()
    ax.grid(True)

plt.suptitle('Average Ankle Torque Profiles During Stance')
plt.tight_layout()
plt.show()