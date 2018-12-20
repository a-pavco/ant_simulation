#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def talker():
	pub = rospy.Publisher('/ant_topic', Float32, queue_size=1)
	rospy.init_node('ant_rosnode_py', anonymous=True)
	rate = rospy.Rate(1) 
	position = Float32()
	tmp = 0;
	position.data = -100
	while not rospy.is_shutdown():
		#rospy.loginfo('Moving ant joints to position == '+str(position))
		pub.publish(position)
		tmp += 1;
		rospy.loginfo(tmp)
		rate.sleep()
		if tmp == 2:
			position.data *= -1
			tmp = 0

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass