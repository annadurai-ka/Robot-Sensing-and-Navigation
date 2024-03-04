#!/usr/bin/env python3
import rospy
import serial
import rosbag
import numpy as np
import time
from vn_driver.msg import Vectornav
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

def ReadFromSerial(serialport):
    # serialport=serial.Serial(SerialPortAddr,rospy.get_param("~baud","115200"))
    vnymrRead=serialport.readline().decode('utf-8').strip()
    # serialport.close()
    return vnymrRead

def convert_to_quaternion(roll, pitch, yaw):
    roll=roll*np.pi/180
    pitch=pitch*np.pi/180
    yaw=yaw*np.pi/180
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

if __name__== '__main__':
    pub = rospy.Publisher('/imu',Vectornav, queue_size=10)
    rospy.init_node('Vectornav_node', anonymous=True)
    # rate = rospy.Rate(40)
    serialPortAddr = rospy.get_param("~port","/dev/ttyUSB0")
    serialport=serial.Serial(serialPortAddr,rospy.get_param("~baud","115200"))
    command = b'$VNWRG,07,40*XX\r\n'
    serialport.write(command)

    while not rospy.is_shutdown():
        vnymrRead=ReadFromSerial(serialport)

        if '$VNYMR' in vnymrRead:

            try:
                vnymrRead=vnymrRead[:-3]
                vnymrSplit=list(vnymrRead.split(","))

                print(vnymrSplit)

                yaw=float(vnymrSplit[1])
                pitch=float(vnymrSplit[2])
                roll=float(vnymrSplit[3])
                Quaternion= convert_to_quaternion(roll, pitch, yaw)
                
                Mx=float(vnymrSplit[4])
                My=float(vnymrSplit[5])
                Mz=float(vnymrSplit[6])
                Ax=float(vnymrSplit[7])
                Ay=float(vnymrSplit[8])
                Az=float(vnymrSplit[9])
                Gx=float(vnymrSplit[10])
                Gy=float(vnymrSplit[11])
                Gz=float(vnymrSplit[12])
                
                msg = Vectornav()
                msg.header= Header()
                msg.header.frame_id = "imu1_frame"
                current_time= rospy.Time.now()
                msg.header.stamp.secs = current_time.secs
                msg.header.stamp.nsecs = current_time.nsecs
                
                imu = Imu()
                mgntcfld= MagneticField()
                imu.orientation.x=Quaternion[0]
                imu.orientation.y=Quaternion[1]
                imu.orientation.z=Quaternion[2]
                imu.orientation.w=Quaternion[3]
                imu.linear_acceleration.x=Ax
                imu.linear_acceleration.y=Ay
                imu.linear_acceleration.z=Az
                imu.angular_velocity.x=Gx
                imu.angular_velocity.y=Gy
                imu.angular_velocity.z=Gz
                mgntcfld.magnetic_field.x=Mx
                mgntcfld.magnetic_field.y=My
                mgntcfld.magnetic_field.z=Mz
                msg.imu = imu
                msg.mag_field= mgntcfld
                msg.vnymr_Read= vnymrRead

                pub.publish(msg)


                # rate.sleep()
                
                print("------------------------------------------")
                print('Reading:', vnymrRead)
                print('IMU x: ', Quaternion[0])
                print('IMU y: ', Quaternion[1])
                print('IMU z: ',Quaternion[2])
                print('IMU w: ', Quaternion[3])
                print('MFx',Mx)
                print('MFy',My)
                print('MFz',Mz)
                print('Acc x',Ax)
                print('Acc y',Ay)
                print('Acc z',Az)
                print('Gyr x',Gx)
                print('Gyr y',Gy)
                print('Gyr z',Gz)
                print("------------------------------------------")


            except:
                continue
    print("--------------------------------End Task ---------------------------------------------------------") 
