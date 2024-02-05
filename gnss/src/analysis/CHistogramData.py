import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
# Read the CSV file into a pandas DataFrame
# df = pd.read_csv("/home/rohit/gnss/Walking200m.csv")
df = pd.read_csv("/home/rohit/gnss/Stationary5min.csv")
#Occluded 
df1 = pd.read_csv("/home/rohit/gnss/Stationary5minsOc.csv")
# Display the first few rows of the DataFrame to understand its structure
print(df.head())
print(df1.head())
# Extract easting and northing columns from the DataFrame
easting = df['UTM_Easting']
northing = df['UTM_Northing']
easting1 = df1['UTM_Easting']
northing1 = df1['UTM_Northing']
# Calculate centroid
centroid_easting = np.mean(easting)
centroid_northing = np.mean(northing)
centroid_easting1 = np.mean(easting1)
centroid_northing1 = np.mean(northing1)

# Calculate Euclidean distances
distances = np.sqrt((easting - centroid_easting)*2 + (northing - centroid_northing)*2)
distances1 = np.sqrt((easting1 - centroid_easting1)*2 + (northing1 - centroid_northing1)*2)
print("Eculidean Distance (open):", distances)
print("Eculidean Distance (occluded):", distances1)
#Plot histogram of Euclidean distances
plt.hist(distances, bins=20, color='skyblue',edgecolor='black', alpha=0.7,label="Stationary Open")
plt.axvline(np.mean(distances), color='blue', linestyle='dashed', linewidth=2, label='Mean Distance of open')

plt.hist(distances1, bins=20, color='red',edgecolor='black', alpha=0.3,label="Stationary Occluded")
plt.axvline(np.mean(distances1), color='red', linestyle='dashed', linewidth=2, label='Mean Distance of Occluded')
plt.xlabel('Euclidean Distance to Centroid (m)')
plt.ylabel('Frequency (Hz)')
plt.title('Histogram of Euclidean Distances to Centroid - occluded')
plt.legend()

# Show the plot
plt.grid(True)
plt.show()