#Stationary both plot together
import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a pandas DataFrame
df = pd.read_csv("/home/rohit/gnss/Walking200m.csv")#/home/sharmila/gnss/Stationary5min.csv

# Display the first few rows of the DataFrame to understand its structure
print(df.head())


# Extract the first point from each dataset
first_point = df.iloc[0]


# Subtract the coordinates of the first point from all points in the dataset
df['dif_Easting'] = df['UTM_Easting'] - first_point['UTM_Easting']
df['dif_Northing'] = df['UTM_Northing'] - first_point['UTM_Northing']

# Find the centroid of the adjusted dataset
centroid_easting = df['dif_Easting'].mean()
centroid_northing = df['dif_Northing'].mean()


# # Calculate the deviation in easting and northing from the centroid
# df['deviation_easting'] = df['dif_Easting'] - centroid_easting
# df['deviation_northing'] = df['dif_Northing'] - centroid_northing

# Print the centroid and deviations
print("Centroid of adjusted dataset:")
print("Easting:", centroid_easting)
print("Northing:", centroid_northing)


# Print the first few rows of the DataFrame with adjusted columns
print(df.head())


# Subtract the centroid from each data point
df['dif_Easting'] -= centroid_easting
df['dif_Northing'] -= centroid_northing


# Plot the adjusted data
plt.figure(figsize=(8, 6))
plt.scatter(df['dif_Easting'], df['dif_Northing'], color='green', alpha=0.5,label="Walking 200m")
plt.xlabel('Difference in Easting (m)')
plt.ylabel('Difference in Northing (m)')
plt.title('MovingData ScatterPlots')
# Annotate the centroid values on the plot
plt.text(centroid_easting, centroid_northing, f'Centroid_Moving: ({centroid_easting:.2f}, {centroid_northing:.2f})',
         fontsize=10, ha='right', va='top', color='blue')

plt.legend()
plt.grid(True)
plt.show()