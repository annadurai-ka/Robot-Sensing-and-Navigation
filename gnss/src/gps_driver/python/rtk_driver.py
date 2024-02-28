#!/usr/bin/env python3
import rospy
import serial
import utm
import time
import rosbag
from std_msgs.msg import Header
from gps_driver.msg import Customrtk


def ReadFromSerial(serialPortAddr):
        serialport=serial.Serial(serialPortAddr,rospy.get_param("~baud", "4800"))
        try:
            gnggaRead=serialport.readline().decode('utf-8').strip()
            serialport.close()
        except:
            gnggaRead=serialport.readline().decode('utf-8').strip()
            serialport.close()
        print(gnggaRead)
        return gnggaRead

def dirsign(latorlong,latorlongdir):
    if latorlongdir == 'S' or latorlongdir=='W':
        return -latorlong
    else:
        return latorlong

def convUTM(Latdir, Longdir):
    UTMcoord= utm.from_latlon(Latdir,Longdir)
    UTMEasting= UTMcoord[0]
    UTMNorthing= UTMcoord[1]
    UTMZone=UTMcoord[2]
    UTMLetter="'"+UTMcoord[3]+"'"
    #print(UTMcoord)
    return[UTMEasting, UTMNorthing, UTMZone, UTMLetter]
    
def UTCtoEpoch(UTC):
    UTCinSecs = (UTC // 10000)*3600 + ((UTC // 100) %  100) * 60 + (UTC % 100) 
    TimeSinceEpoch = time.time() 
    TimeSinceEpochBOD = TimeSinceEpoch - (TimeSinceEpoch % 86400) 
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime) 
    CurrentTimeNsec = int((CurrentTime - CurrentTimeSec) * 1e9) 
    #print(CurrentTimeSec)
    #print(CurrentTimeNsec)
    return [CurrentTimeSec, CurrentTimeNsec]

if __name__ == '__main__':
    pub = rospy.Publisher('rtk_gnss',Customrtk, queue_size=10)
    rospy.init_node('rtk_gnss', anonymous=True)
    
    rate = rospy.Rate(10)
    rospy.loginfo("Publishing....................................")

    while not rospy.is_shutdown():  
        serialPortAddr = rospy.get_param("~port","/dev/pts/4")
        gnggaRead=ReadFromSerial(serialPortAddr)
        #print('2')
        #gnggaRead='$gngga,202530.00,5109.0262,N,11401.8407,W,5,40,0.5,1097.36,M,-17.00,M,18,TSTR*61'
                
        if '$GNGGA' in gnggaRead:
            try:
                gnggaSplit = list(gnggaRead.split(","))
                
                print(gnggaSplit)
            
                UTC = float(gnggaSplit[1])
                Latitude = float(gnggaSplit[2])//100 + (float(gnggaSplit[2])%100)/60
                LatitudeDir = str(gnggaSplit[3])
                Longitude = float(gnggaSplit[4])//100 + (float(gnggaSplit[4])%100)/60
                LongitudeDir = str(gnggaSplit[5])
                fix_quality = int(gnggaSplit[6])
                HDOP = float(gnggaSplit[8])

                Latitudeg=dirsign(Latitude,LatitudeDir)
                Longitudeg=dirsign(Longitude,LongitudeDir)
                print("2")
                UTMVal = convUTM(Latitudeg, Longitudeg)
                CurrentTime= UTCtoEpoch(UTC)
                msg = Customrtk()
                msg.header = Header()
                msg.header.frame_id = "GPS1_frame"
                msg.header.stamp.secs= CurrentTime[0]
                msg.header.stamp.nsecs= CurrentTime[1]
                msg.latitude= Latitudeg
                msg.longitude= Longitudeg
                msg.altitude = float(gnggaSplit[9])
                msg.utm_easting = UTMVal[0]
                msg.utm_northing = UTMVal[1]
                msg.zone = UTMVal[2]
                msg.letter= UTMVal[3]
                msg.fix_quality = fix_quality
                msg.hdop=HDOP
                msg.gngga_read= gnggaRead
                
                pub.publish(msg)
    
                

                rate.sleep()
                print("------------------------------------------")
                print('Reading:', gnggaRead)
                print()
                print("UTC: " +str(UTC))
                print('Latitude: ', gnggaSplit[2])
                print('LatitudeDir: ', LatitudeDir)
                print('Longitude: ', gnggaSplit[4])
                print('LongitudeDir: ', LongitudeDir)
                print('HDOP: ', HDOP)
                print()
                print('LatitudeSigned: ',Latitudeg)
                print('LongitudeSigned: ', Longitudeg)
                print('CurrentTimeSec: ', CurrentTime[0])
                print('CurrentTimeNsec:', CurrentTime[1])
                print()
                print('EASTING: ', UTMVal[0])
                print('NORTHING: ', UTMVal[1])
                print('ZONE_NUMBER: ',UTMVal[2])
                print('ZONE_LETTER: ', UTMVal[3])
                print("------------------------------------------")
            except:
                continue
            
    print("Saving .bag file")
            


            



