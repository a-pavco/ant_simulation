#!/usr/bin/env python

from threading import Thread
import rospy
import math
from ant_gazebo.ant import Ant

# joint limits as specified by ant.xacro
f1_low = -0.4
f1_upp = 0.6
m1_low = -0.65
m1_upp = 0.15
r1_low = -0.4
r1_upp = 0.25

j2_low = 0.0
j2_upp = 0.5
r2_low = 0.15

j3_low = 0
j3_upp = 0.5

class JointFunc:
	def __init__(self, low, upp, 
		invert, scalephase, j2=False):

		self.j2 = j2
		self.low = low
		self.upp = upp
		self.invert = invert
		self.scalephase = scalephase

	def get(self, x):
		if self.j2:
			if x < 0.5:
				return self.scalephase * self.invert * (
					x / 0.5   * (self.upp - self.low) + self.low)
			else:
				return -1 * self.scalephase * self.invert * (
					(x - 0.5) / (0.5)  * (-self.low - -self.upp) + -self.upp)
		else:
			return self.invert * (x * (self.upp - self.low) + self.low)

		# (x - oldlow) / (oldupp - oldlow) ) * (0.6 - -0.3) + -0.3

class WalkFunc:
	def __init__(self):
		self.generate()

	def generate(self):
		self.phase = {}  # phase functions
		self.antph = {}  # anti phase functions

		self.phase['j_1_f_l'] = JointFunc(-f1_upp, -f1_low, -1, 1)
		self.antph['j_1_f_l'] = JointFunc(f1_low, f1_upp, 1, 1)
		self.phase['j_1_f_r'] = JointFunc(f1_low, f1_upp, -1, 1)
		self.antph['j_1_f_r'] = JointFunc(-f1_upp, -f1_low, 1, 1)
		self.phase['j_1_m_l'] = JointFunc(m1_low, m1_upp, 1, 1)
		self.antph['j_1_m_l'] = JointFunc(-m1_upp, -m1_low, -1, 1)
		self.phase['j_1_m_r'] = JointFunc(-m1_upp, -m1_low, 1, 1)
		self.antph['j_1_m_r'] = JointFunc(m1_low, m1_upp, -1, 1)
		self.phase['j_1_r_l'] = JointFunc(-r1_upp, -r1_low, -1, 1)
		self.antph['j_1_r_l'] = JointFunc(r1_low, r1_upp, 1, 1)
		self.phase['j_1_r_r'] = JointFunc(r1_low, r1_upp, -1, 1)
		self.antph['j_1_r_r'] = JointFunc(-r1_upp, -r1_low, 1, 1)

	
		self.phase['j_2_f_l'] = JointFunc(j2_low, j2_upp, 1, 1, True)
		self.antph['j_2_f_l'] = JointFunc(j2_low, j2_upp, 1, 0, True)
		self.phase['j_2_m_r'] = JointFunc(j2_low, j2_upp, -1, 1, True)
		self.antph['j_2_m_r'] = JointFunc(j2_low, j2_upp, -1, 0, True)
		self.phase['j_2_r_l'] = JointFunc(r2_low, j2_upp, 1, 1, True)
		self.antph['j_2_r_l'] = JointFunc(r2_low, j2_upp, 1, 0, True)

		self.phase['j_2_f_r'] = JointFunc(j2_low, j2_upp, -1, 0, True)
		self.antph['j_2_f_r'] = JointFunc(j2_low, j2_upp, -1, 1, True)
		self.phase['j_2_m_l'] = JointFunc(j2_low, j2_upp, 1, 0, True)
		self.antph['j_2_m_l'] = JointFunc(j2_low, j2_upp, 1, 1, True)
		self.phase['j_2_r_r'] = JointFunc(r2_low, j2_upp, -1, 0, True)
		self.antph['j_2_r_r'] = JointFunc(-r2_low, j2_upp, -1, 1, True)

		self.phase['j_3_f_l'] = JointFunc(j3_low, j3_upp, 1, 1)
		self.antph['j_3_f_l'] = JointFunc(-j3_upp, j3_low, -1, 1)

		self.phase['j_3_f_r'] = JointFunc(-j3_upp, j3_low, -1, 1)
		self.antph['j_3_f_r'] = JointFunc(j3_low, j3_upp, 1, 1)

		self.phase['j_3_m_l'] = JointFunc(-j3_upp, j3_low, -1, 1)
		self.antph['j_3_m_l'] = JointFunc(j3_low, j3_upp, 1, 1)

		self.phase['j_3_m_r'] = JointFunc(j3_low, j3_upp, 1, 1)
		self.antph['j_3_m_r'] = JointFunc(-j3_upp, j3_low, -1, 1)

		self.phase['j_3_r_l'] = JointFunc(-j3_upp, j3_low, -1, 1)
		self.antph['j_3_r_l'] = JointFunc(j3_low, j3_upp, 1, 1)

		self.phase['j_3_r_r'] = JointFunc(j3_low, j3_upp, 1, 1)
		self.antph['j_3_r_r'] = JointFunc(-j3_upp, j3_low, -1, 1)

	def get(self, phase, x):
		angles = {}
		for j in self.phase.keys():
			if phase:
				v = self.phase[j].get(x)
				angles[j] = v
			else:
				angles[j] = self.antph[j].get(x)
		return angles

class Walk:
	def __init__(self, robot):
		self.robot = robot
		self.func = WalkFunc()
		self.thread = None

	def start(self):
		self.thread = Thread(target=self.walk)
		self.thread.start()

	def walk(self):
		r = rospy.Rate(50)
		rospy.loginfo('Started walking thread')
		func = self.func
		n = 25
		p = True
		#rospy.loginfo("Phase: " + str(p))
		i = 0
		while (not rospy.is_shutdown()):
			x = float(i) / n
			angles = func.get(p, x)
			#rospy.loginfo(angles)
			#rospy.loginfo(i)
			self.robot.set_angles(angles)
			i += 1
			if i > n:
				i = 0
				p = not p
				#rospy.loginfo('Changing phase, sleeping 1 sec.')
				#rospy.sleep(1)
				#rospy.loginfo("Phase: " + str(p))
			r.sleep()
		self.thread = None


if __name__ == '__main__':
	rospy.init_node('walk')
	rospy.sleep(1)

	robot = Ant()

	#rospy.loginfo(robot.get_angles())

	walk = Walk(robot)
	
	rospy.loginfo('Sleeping 1 sec')
	rospy.sleep(1)
	rospy.loginfo('Starting walking')
	walk.start()

	while not rospy.is_shutdown():
		rospy.sleep(1)
