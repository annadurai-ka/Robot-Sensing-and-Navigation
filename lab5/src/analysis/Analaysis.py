#!/usr/bin/env python
from bagpy import bagreader
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import stats, signal
from math import cos, sin
from scipy.spatial.transform import Rotation as R

imu_frequency = 40
gps_frequency = 1

def extract_data(filepath, topic):
    bag = bagreader(filepath)
    dataset = bag.message_by_topic(topic=topic)
    dataset = pd.read_csv(dataset)
    return dataset

def calculate_periodic_indices(dataset, period, frequency):
    length = len(dataset)
    print(length)
    initial_index = int(length // 2)
    final_index = int(initial_index + period * 3 * frequency)
    print(initial_index)
    print(final_index)
    t0 = 1/imu_frequency 
    print(min(final_index, len(dataset) - 30))
    indices = [0]
    rates = np.ones(30)

    for idx in range(initial_index, min(final_index, len(dataset) - 30)):
        for j in range(30):
            rates[j] = np.diff(dataset)[idx-j] /t0
        optimal_index = idx - np.argmin(abs(rates))
        if abs(np.mean(rates)) < 0.01 and (optimal_index - indices[-1]) > 200:
            indices.append(optimal_index)

    return indices

def compute_calibration_values(magnetic_data):
    mean_x, mean_y = magnetic_data[0].mean(), magnetic_data[1].mean()
    magnetic_data[0] -= mean_x
    magnetic_data[1] -= mean_y
    U, S, V = np.linalg.svd(np.stack((magnetic_data[0], magnetic_data[1])))
    num_points = len(magnetic_data[0])
    theta = np.linspace(0, 2 * np.pi, num_points)
    unit_circle = np.stack((np.cos(theta), np.sin(theta)))
    transformation = np.sqrt(2 / num_points) * U.dot(np.diag(S))
    fit = transformation.dot(unit_circle) + np.array([[0], [0]])
    angle_a = np.argmax(fit[0, :])
    angle_b = np.argmax(fit[1, :])
    rotation = math.atan2(fit[1, angle_a], fit[0, angle_a])
    semi_major = math.sqrt(fit[1, angle_a]**2 + fit[0, angle_a]**2)
    semi_minor = math.sqrt(fit[1, angle_b]**2 + fit[0, angle_b]**2)

    magnetic_data[0] += mean_x
    magnetic_data[1] += mean_y

    return mean_x, mean_y, rotation, semi_major, semi_minor

def apply_calibration(magnetic_data, center_x, center_y, angle, major, minor):
    adjusted_data = [magnetic_data[0], magnetic_data[1]]
    adjusted_data[0] = magnetic_data[0] - center_x
    adjusted_data[1] = magnetic_data[1] - center_y
    adjusted_data[0] = cos(angle) * adjusted_data[0] + sin(angle) * adjusted_data[1]
    adjusted_data[1] = -sin(angle) * adjusted_data[0] + cos(angle) * adjusted_data[1]
    adjusted_data[0] *= minor / major
    return adjusted_data

def apply_butterworth_filter(data, sample_rate, cutoff, filter_type):
    nyquist = 0.5 * sample_rate
    normal_cutoff = cutoff / nyquist
    sos = signal.butter(2, normal_cutoff, btype=filter_type, output='sos', analog=False)
    filtered_data = signal.sosfilt(sos, data)
    return filtered_data

def compute_moving_average(data, window_size=3):
    cumulative_sum = np.cumsum(data, dtype=float)
    cumulative_sum[window_size:] = cumulative_sum[window_size:] - cumulative_sum[:-window_size]
    return cumulative_sum[window_size - 1:] / window_size

def detect_stationary_periods(acceleration_data, stationary_duration, frequency):
    threshold = stationary_duration * frequency
    stationary_indices = []
    zero_change_count = 0
    rate_of_change = np.diff(acceleration_data)

    for idx in range(1, len(rate_of_change)):
        previous_count = zero_change_count
        if abs(rate_of_change[idx]) < 0.05:
            zero_change_count += 1
        else:
            zero_change_count = 0

        if previous_count > threshold and zero_change_count == 0:
            stationary_indices.append(idx - 1 - previous_count)
            stationary_indices.append(idx - 1)

    return stationary_indices

def calculate_gps_velocity_and_displacement(gps_data):
    number_of_samples = len(gps_data)
    velocities = np.ones(number_of_samples)
    displacements = np.zeros(number_of_samples)
    initial_x = gps_data['utm_easting'][0]
    initial_y = gps_data['utm_northing'][0]

    for idx in range(1, number_of_samples):
        previous_x = gps_data['utm_easting'][idx - 1]
        previous_y = gps_data['utm_northing'][idx - 1]
        current_x = gps_data['utm_easting'][idx]
        current_y = gps_data['utm_northing'][idx]
        velocities[idx - 1] = math.sqrt((current_y - previous_y)**2 + (current_x - previous_x)**2)
        displacements[idx] = math.sqrt((current_y - initial_y)**2 + (current_x - initial_x)**2)

    velocities[-1] = velocities[-2]
    second_velocities = np.diff(displacements)
    np.append(second_velocities, 0)
    return velocities, displacements, second_velocities

print("Execution started")
calibration_filepath = '/home/kaviak/catkin_ws/CircleVectornavBAG.bag'
imu_dataset = extract_data(calibration_filepath, 'imu')
print(imu_dataset.columns)
magnetic_x = imu_dataset['mag_field.magnetic_field.x']
magnetic_y = imu_dataset['mag_field.magnetic_field.y']

periodic_indices = calculate_periodic_indices(magnetic_x, 20, imu_frequency)
magnetic_data = [magnetic_x, magnetic_y]
print(magnetic_data[0])
print(magnetic_data[1])
calibration_results = compute_calibration_values(magnetic_data)
print(calibration_results)

calibrated_magnetic_data = apply_calibration(magnetic_data, *calibration_results)
plt.figure(figsize=(100, 60))
plt.scatter(calibrated_magnetic_data[0], calibrated_magnetic_data[1], color='red', marker='o')
plt.scatter(magnetic_data[0], magnetic_data[1], color='blue', marker='x')
plt.legend(['Magnetometer calibrated', 'Magnetometer measured'])
plt.xlabel('Magnetic_field X (Gauss)')
plt.ylabel('Magnetic_field Y (Gauss)')
plt.margins(0.005)
plt.grid(True)
plt.show()
print("Execution continues")
moving_filepath = '/home/kaviak/catkin_ws/BostonVectornavBAG.bag'
imu_dataset = extract_data(moving_filepath, 'imu')
print(imu_dataset.columns)
imu_times = imu_dataset['header.stamp.secs'] - imu_dataset['header.stamp.secs'].min() + (imu_dataset['header.stamp.nsecs'] // 10**6) / 1000
num_imu_samples = len(imu_times)
imu_dataset['IMU.yaw'] = np.ones(num_imu_samples)
imu_dataset['IMU.pitch'] = np.ones(num_imu_samples)
imu_dataset['IMU.roll'] = np.ones(num_imu_samples)

for index in range(num_imu_samples):
    split_data = imu_dataset['vnymr_Read'][index].split(',')
    imu_dataset.loc[index, 'IMU.yaw'] = float(split_data[1])
    imu_dataset.loc[index, 'IMU.pitch'] = float(split_data[2])
    imu_dataset.loc[index, 'IMU.roll'] = float(split_data[3])
    last_part = split_data[-1].split('*')
    imu_dataset.loc[index, 'imu.angular_velocity.z'] = float(last_part[0])

imu_dataset['IMU.pitch'] = imu_dataset['IMU.pitch'] - np.mean(imu_dataset['IMU.pitch'][:50])
imu_dataset['IMU.roll'] = imu_dataset['IMU.roll'] - np.mean(imu_dataset['IMU.roll'][:50])
magnetic_data = [imu_dataset['mag_field.magnetic_field.x'], imu_dataset['mag_field.magnetic_field.y']]
calibrated_magnetic_data = apply_calibration(magnetic_data, *calibration_results)

imu_dataset['IMU.yaw'] = np.unwrap(imu_dataset['IMU.yaw'])
imu_dataset['IMU.yaw'] = imu_dataset['IMU.yaw'] - np.mean(imu_dataset['IMU.yaw'][:20])
uncalibrated_yaw = np.ones(num_imu_samples)
calibrated_yaw = np.ones(num_imu_samples)

for index in range(num_imu_samples):
    uncalibrated_yaw[index] = -math.degrees(math.atan2(magnetic_data[1][index], magnetic_data[0][index]))
    calibrated_yaw[index] = -math.degrees(math.atan2(calibrated_magnetic_data[1][index], calibrated_magnetic_data[0][index]))

average_window = 50
correction_start = 19000
correction_end = 26500
calibrated_yaw[correction_start:correction_end-average_window+1] = compute_moving_average(calibrated_yaw[correction_start:correction_end], average_window)
uncalibrated_yaw[correction_start:correction_end-average_window+1] = compute_moving_average(uncalibrated_yaw[correction_start:correction_end], average_window)

average_window = 100
calibrated_yaw = np.unwrap(calibrated_yaw, period=360)
uncalibrated_yaw = np.unwrap(uncalibrated_yaw, period=360)
plt.plot(imu_times, calibrated_yaw, color='red')
plt.plot(imu_times, uncalibrated_yaw, color='green')
plt.legend(['Magnetometer yaw after calibration', 'Magnetometer yaw before calibration'])
plt.xlabel('Time (sec)')
plt.ylabel('Yaw Orientation (deg)')
plt.grid(True)
plt.show()

filtered_yaw = apply_butterworth_filter(calibrated_yaw, imu_frequency, 0.08, 'low')
gyro_yaw = np.cumsum(imu_dataset['imu.angular_velocity.z']) * 1 / imu_frequency
for index in range(num_imu_samples):
    gyro_yaw[index] = math.degrees(math.atan2(math.sin(gyro_yaw[index]), math.cos(gyro_yaw[index])))
gyro_yaw = np.unwrap(gyro_yaw)
plt.plot(imu_times, gyro_yaw, color='cyan')
plt.legend(['Gyro yaw estimation'])
plt.xlabel('Time (sec)')
plt.ylabel('Yaw Orientation (deg)')
plt.grid(True)
plt.show()

alpha = 0.2
complementary_yaw = alpha * filtered_yaw + (1 - alpha) * apply_butterworth_filter(gyro_yaw, imu_frequency, 0.0000001, 'high')
plt.plot(imu_times, imu_dataset['IMU.yaw'], color='cyan')
plt.plot(imu_times, complementary_yaw, color='green')
plt.legend(['IMU yaw heading', 'Complementary filter'])
plt.xlabel('Time (sec)')
plt.ylabel('Yaw Orientation (deg)')
plt.grid(True)
plt.show()

gps_data = extract_data(moving_filepath, 'gps')
gps_times = gps_data['header.stamp.secs'] - gps_data['header.stamp.secs'].min()
gps_data['utm_easting'] = gps_data['utm_easting'] - gps_data['utm_easting'][0]
gps_data['utm_northing'] = gps_data['utm_northing'] - gps_data['utm_northing'][0]
gps_velocities, gps_displacements, gps_second_velocities = calculate_gps_velocity_and_displacement(gps_data)

imu_accelerations = imu_dataset['imu.linear_acceleration.x'] - imu_dataset['imu.linear_acceleration.x'].mean()
imu_velocities = np.cumsum(imu_accelerations * 1 / imu_frequency)
stationary_indices = detect_stationary_periods(imu_accelerations, 2, imu_frequency)
imu_accelerations = imu_accelerations - np.mean(imu_accelerations[stationary_indices])

plt.plot(gps_times, gps_velocities, '-', color='magenta')
plt.legend(['Forward Velocity from GPS'])
plt.xlabel('Velocity (m/sec)')
plt.ylabel('Time (sec)')
plt.grid(True)
plt.show()

print("Processing complete.")
