#moving data
import pandas as pd
import matplotlib.pyplot as plt
df = pd.read_csv("/home/rohit/gnss/Walking200m.csv")
df['Time']= df['Seconds']
print(df['Time'])
# Plot 
plt.figure(figsize=(100, 100))
plt.scatter(df['Time'],df['Altitude'],  color='green', alpha=0.5,label="Walking 200m")
plt.xlabel('Time (s)')
plt.ylabel('Moving Altitude (m)')
plt.legend()
plt.title('Plot of Moving Altitude VS Time')
plt.grid(True)
plt.show()