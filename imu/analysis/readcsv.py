import sys
import os
import os.path
import csv
import rosbag


bag=rosbag.Bag('/home/kaviak/catkin_ws/VectornavBAG.bag')
# for topic,msg, t in bag.read_messages(topics=['chatter']):
#     print(msg)
#     print(t)
# bag.close()



dir = '/home/kaviak/catkin_ws/'
if not os.path.exists(dir):
    os.mkdir(dir)

#with open(results_dir +"/"+filename+'testcsv.csv', mode='w') as data_file:
with open(os.path.join(dir, "testcsv"+'.csv'), "w") as data_file:
    data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    data_writer.writerow(['Seconds','NanoSeconds','w', 'x', 'y', 
    'z', 'mag field x', 'mag field y', 'mag field z','lin acc x','lin acc y','lin acc z','ang vel x','ang vel y','ang vel z'])
    # Get all message on the /joint states topic

    for topic, msg, t in bag.read_messages(topics=['imu']):
# Only write to CSV if the message is for our robot
        data_writer.writerow([msg.header.stamp.secs,msg.header.stamp.nsecs,msg.imu.orientation.w, msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.mag_field.magnetic_field.x, msg.mag_field.magnetic_field.y, msg.mag_field.magnetic_field.z,  msg.imu.linear_acceleration.x,msg.imu.linear_acceleration.y,msg.imu.linear_acceleration.z,msg.imu.angular_velocity.x,msg.imu.angular_velocity.y, msg.imu.angular_velocity.z])
print(msg.header.stamp.secs)        
print("Finished creating csv file!")
bag.close()

