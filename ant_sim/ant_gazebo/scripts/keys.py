#!/usr/bin/env python
import getch
import rospy 
from std_msgs.msg import Int8

def keys():
    pub = rospy.Publisher('key', Int8, queue_size = 10)  
    rospy.init_node('keypress', anonymous = True)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        k = ord(getch.getch())
        #rospy.loginfo(str(k))
        if ((k >= 65) & (k <= 68) | (k == 114)
            | (k == 122) | (k == 120) | (k == 99)):
            pub.publish(k)
            #rospy.loginfo(str(k))
        rate.sleep()
        
if __name__=='__main__':
    try:
        keys()
    except rospy.ROSInterruptException:
        pass