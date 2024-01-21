#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
	message1 = data.data[::-1]
	rospy.loginfo("I heard that %s", message1)
	
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('chatter', String, callback)
	rospy.spin()
	
if __name__ == '__main__':
	listener()
	
