#!/usr/bin/env python
import getch
import rospy 
from std_msgs.msg import Int8

def keys():
    while not rospy.is_shutdown():
        k = ord(getch.getch())
        #rospy.loginfo(str(k))
        if ((k >= 65) & (k <= 68) | (k == 114)
            | (k == 122) | (k == 120) | (k == 99)):
            pub.publish(k)
            #rospy.loginfo(str(k))
        rate.sleep()
        
if __name__=='__main__':
    rospy.init_node('keys_node', anonymous = True)
    pub = rospy.Publisher('/ant/key', Int8, queue_size = 1)  
    rate = rospy.Rate(100)
    try:
        keys()
    except rospy.ROSInterruptException:
        pass