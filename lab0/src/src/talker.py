#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def talker():
	publisher = rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1)
	
	while not rospy.is_shutdown():
		message = "Ros is running smoothly and so I gonna drink smoothy"
		rospy.loginfo(message)
		publisher.publish(message)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass