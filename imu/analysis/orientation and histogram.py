import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def quaternion_to_euler(w, x, y, z):
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1, 1))

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

# Load quaternion data from CSV file
data = pd.read_csv('/home/kaviak/catkin_ws/testcsv.csv')
data['time'] = data['Seconds'] + data['NanoSeconds'] * 1e-9
print('Time: ', data['time'].values)

# Convert quaternion components to Euler angles
data['roll'], data['pitch'], data['yaw'] = quaternion_to_euler(data['w'], data['x'], data['y'], data['z'])

# Create subplots
fig, axs = plt.subplots(3, 1, figsize=(10, 10))

# Plot x-axis rotation (roll)
axs[0].plot(data['time'].values, data['roll'].values, 'bo', label='Roll')
axs[0].set_ylabel('Roll (deg)')
axs[0].legend()

# Plot y-axis rotation (pitch)
axs[1].plot(data['time'].values, data['pitch'].values, 'ro', label='Pitch')
axs[1].set_ylabel('Pitch (deg)')
axs[1].legend()

# Plot z-axis rotation (yaw)
axs[2].plot(data['time'].values, data['yaw'].values, 'go', label='Yaw')
axs[2].set_ylabel('Yaw (deg)')
axs[2].legend()

# Set common labels
plt.xlabel('Time (sec)')

# Set a main title
plt.suptitle('Rotation from VN Estimation vs Time')
plt.grid(True)
plt.show()

#Histogram

fig, axs = plt.subplots(3, 1, figsize=(10, 10))

# Plot histogram for roll
axs[0].hist(data['roll'], bins=50, color='blue', alpha=0.5,label='Roll')
axs[0].set_ylabel('Frequency(Roll)')
axs[0].legend()

# Plot histogram for pitch
axs[1].hist(data['pitch'], bins=50, color='red', alpha=0.5,label='Pitch')
axs[1].set_ylabel('Frequency(Pitch)')
axs[1].legend()

# Plot histogram for yaw
axs[2].hist(data['yaw'], bins=50, color='green', alpha=0.5,label='Yaw')
axs[2].set_ylabel('Frequency(Yaw)')
axs[2].legend()

# Set a main title for histograms
plt.suptitle('Histograms of Rotation from VN Estimation')
plt.xlabel('(deg)')
plt.show()