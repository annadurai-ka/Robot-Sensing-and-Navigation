import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv('/home/kaviak/catkin_ws/testcsv.csv') 
data['time'] = data['Seconds'] + data['NanoSeconds'] * 1e-9
print('Time: ', data['time'].values)

fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

axs[0].plot(data['time'].values, data['lin acc x'].values, 'b', label='X (blue marker)')
axs[0].set_ylabel('X Acceleration (m/s^2)')
axs[0].legend()


axs[1].plot(data['time'].values, data['lin acc y'].values, 'r', label='Y (red marker)')
axs[1].set_ylabel('Y Acceleration (m/s^2)')
axs[1].legend()


axs[2].plot(data['time'].values, data['lin acc z'].values, 'g', label='Z (green marker)')
axs[2].set_ylabel(' Z Acceleration (m/s^2)')
axs[2].legend()
plt.suptitle('Acceleration from the accelerometer vs Time')
plt.xlabel('Time (s)')
plt.grid(True)
plt.show()