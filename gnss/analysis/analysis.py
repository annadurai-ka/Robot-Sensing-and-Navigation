import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
sns.set_theme()
sns.set(font_scale=3)
plt.rcParams.update({'font.size': 40})

bag = bagreader('/home/kaviak/catkin_ws/OpenArea.bag')
#bag.topic_table
data = bag.message_by_topic('')
readings = pd.read_csv(data)
readings['utm_easting'] = readings['utm_easting'] - readings['utm_easting'].min()
readings['utm_northing'] = readings['utm_northing'] - readings['utm_northing'].min()
print(readings[['utm_easting', 'utm_northing']])
print(readings)
readings[['utm_easting','utm_northing']].plot()
plt.show()
# fig, ax = bagpy.create_fig(1)
sns.scatterplot(x = 'utm_easting', y = 'utm_northing', data = readings, s= 50, label = 'utm_easting VS utm_northing')
plt.xlabel('utm_easting (meters)')
plt.ylabel('utm_northing (meters)')
plt.title('utm_easting vs utm_northing')
plt.show()

