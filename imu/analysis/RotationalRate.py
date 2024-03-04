import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv('/home/kaviak/catkin_ws/testcsv.csv') 

# Convert rad/s to deg/s
data['x_deg_per_s'] = data['ang vel x'] * (180 / np.pi)
data['y_deg_per_s'] = data['ang vel y'] * (180 / np.pi)
data['z_deg_per_s'] = data['ang vel z'] * (180 / np.pi)
data['time'] = data['Seconds'] + data['NanoSeconds'] * 1e-9
print('Time: ', data['time'].values)

fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

axs[0].plot(data['time'].values, data['x_deg_per_s'].values, 'b', label='X (blue marker)')
axs[0].set_ylabel('X Rotational Rate (deg/s)')
axs[0].legend()


axs[1].plot(data['time'].values, data['y_deg_per_s'].values, 'r', label='Y (red marker)')
axs[1].set_ylabel('Y Rotational Rate (deg/s)')
axs[1].legend()


axs[2].plot(data['time'].values, data['z_deg_per_s'].values, 'g', label='Z (green marker)')
axs[2].set_ylabel(' Z Rotational Rate (deg/s)')
axs[2].legend()
plt.suptitle('Rotational Rate from Gyro vs Time')
plt.xlabel('Time (s)')
plt.grid(True)
plt.show()
