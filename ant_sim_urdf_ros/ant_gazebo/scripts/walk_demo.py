#!/usr/bin/env python

import rospy
from ant_gazebo.ant import Ant


if __name__ == '__main__':
    rospy.init_node('walk_demo')

    rospy.loginfo('Instantiating robot Client')
    robot = Ant()
    rospy.sleep(1)

    rospy.loginfo('Walk Demo Starting')
    robot.set_walk_velocity(1, 0, 0)
    rospy.loginfo('Walk Demo Finished')
