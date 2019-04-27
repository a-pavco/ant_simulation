#!/usr/bin/env python
import rospy
from numpy import interp
from threading import Thread
from std_msgs.msg import Int8
from std_srvs.srv import Empty
from ant_gazebo.ant import Ant

# joint limits as specified by ant.xacro
# for the left side from ant perspective

f1_low = -0.4
f1_upp = 0.6
m1_low = -0.65
m1_upp = 0.15
r1_low = -0.4
r1_upp = 0.25

f2_low = 0.05
f2_upp = 0.5
m2_low = -0.13
m2_upp = 0.5
r2_low = 0.0001
r2_upp = 0.5

f3_low = 0
f3_upp = 0.72
m3_low = 0
m3_upp = 0.4
r3_low = 0
r3_upp = 0.4

f2_low_front = -0.3
r2_low_back = -0.15

speeds = [35, 25, 15]

class WalkControl:
	def __init__(self, robot):
		self.robot = robot
		self.func = MotionFunc()
		self.thread = None
		#S - standing still, F - forward 
		#L - turning left, R - turning right
		self.motion = 'S'
		self.phase = True
		self.first_step = True
		self.i = 0
		self.n = speeds[1]
		self.rate = rospy.Rate(50)

		self._sub_keys = rospy.Subscriber(
			"/ant/key", Int8, self.got_key, queue_size = 1)

		rospy.wait_for_service('/gazebo/reset_world')
		self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

	def start(self):
		self.thread = Thread(target = self.walk)
		self.thread.start()

	def walk(self):
		rospy.loginfo('Starting simulation.')
		self.robot.center_angles()
		while (not rospy.is_shutdown()):
			if (self.motion != 'S'):
				x = float(self.i) / self.n
				if (self.motion == 'F'):
					angles = self.func.get_forward(self.phase, self.first_step, x)
				elif (self.motion == 'L'):
					angles = self.func.get_turn(True, self.phase, x)
				else:
					angles = self.func.get_turn(False, self.phase, x)					
				self.robot.set_angles(angles)
				self.i += 1
				if self.i > self.n:
					self.i = 0
					self.first_step = False
					self.phase = not self.phase
					#rospy.loginfo(angles)
					#rospy.loginfo('Changing phase, sleeping 1 sec.')
					#rospy.sleep(1)
					#rospy.loginfo("Phase: " + str(self.phase))
				self.rate.sleep()
		self.thread = None

	def reset_walk(self):
		self.robot.center_angles()
		self.phase = True
		self.first_step = True
		self.i = 0

	def got_key(self, msg):
		#rospy.loginfo("Got key: " +str(msg))
		if msg.data == 65:
			if self.motion == 'S':
				self.motion = 'F'
				rospy.loginfo('Walking forward.')
		elif msg.data == 66:
			if self.motion == 'F':
				self.motion = 'S'
				rospy.loginfo('Stopping and standing.')
				self.reset_walk()
		elif msg.data == 68:
			if self.motion == 'S':
				self.motion = 'L'
				rospy.loginfo('Turning left.')
			elif self.motion == 'R':
				self.motion = 'S'
				rospy.loginfo('Stopping turning right.')
				self.reset_walk()
		elif msg.data == 67:
			if self.motion == 'S':
				self.motion = 'R'
				rospy.loginfo('Turning right.')
			elif self.motion == 'L':
				self.motion = 'S'
				rospy.loginfo('Stopping turning left.')
				self.reset_walk()
		elif msg.data == 114:
			rospy.loginfo('Reseting model postion, standing.')
			self.motion = 'S'
			self.reset_world()
			self.reset_walk()
		elif msg.data == 122:
			self.n = speeds[0]
			rospy.loginfo('Speed set to slow.')
		elif msg.data == 120:
			self.n = speeds[1]
			rospy.loginfo('Speed set to medium.')
		else:
			self.n = speeds[2]
			rospy.loginfo('Speed set to fast.')			

class MotionFunc:
	def __init__(self):
		self.generate_forward()
		self.generate_first_step()
		self.generate_left_turn()
		self.generate_right_turn()

	def get_forward(self, phase, first_step, x):
		angles = {}
		for j in self.forward_ph.keys():
			if first_step:
				angles[j] = self.first_stp[j].get(x)
			elif phase:
				angles[j] = self.forward_ph[j].get(x)
			else:
				angles[j] = self.forward_ap[j].get(x)
		return angles

	def get_turn(self, left, phase, x):
		angles = {}
		for j in self.forward_ph.keys():
			if 'j_2' in j or 'j_3' in j:
				if phase:
					angles[j] = self.forward_ph[j].get(x)
				else:
					angles[j] = self.forward_ap[j].get(x)
			else:
				if left:
					if phase:
						angles[j] = self.left_ph[j].get(x)
					else:
						angles[j] = self.left_ap[j].get(x)
				else:
					if phase:
						angles[j] = self.right_ph[j].get(x)
					else:
						angles[j] = self.right_ap[j].get(x)		
		return angles

	def generate_forward(self):
		self.forward_ph = {}  # forward phase functions
		self.forward_ap = {}  # forward anti phase functions

		self.forward_ph['j_1_f_l'] = Joint(-f1_upp, -f1_low, -1)
		self.forward_ap['j_1_f_l'] = Joint(f1_low, f1_upp, 1)
		self.forward_ph['j_1_f_r'] = Joint(f1_low, f1_upp, -1)
		self.forward_ap['j_1_f_r'] = Joint(-f1_upp, -f1_low, 1)
		self.forward_ph['j_1_m_l'] = Joint(m1_low, m1_upp, 1)
		self.forward_ap['j_1_m_l'] = Joint(-m1_upp, -m1_low, -1)
		self.forward_ph['j_1_m_r'] = Joint(-m1_upp, -m1_low, 1)
		self.forward_ap['j_1_m_r'] = Joint(m1_low, m1_upp, -1)
		self.forward_ph['j_1_r_l'] = Joint(-r1_upp, -r1_low, -1)
		self.forward_ap['j_1_r_l'] = Joint(r1_low, r1_upp, 1)
		self.forward_ph['j_1_r_r'] = Joint(r1_low, r1_upp, -1)
		self.forward_ap['j_1_r_r'] = Joint(-r1_upp, -r1_low, 1)

	
		self.forward_ph['j_2_f_l'] = JointAmplit(f2_low, f2_upp, 1, 1, f2_low_front)
		self.forward_ap['j_2_f_l'] = Joint(f2_low_front, f2_low, 1)
		self.forward_ph['j_2_m_r'] = JointAmplit(m2_low, m2_upp, -1, 1)
		self.forward_ap['j_2_m_r'] = JointAmplit(-m2_low, -m2_low, 1, 1)
		self.forward_ph['j_2_r_l'] = JointAmplit(r2_low_back, r2_upp, 1, 1, r2_low)
		self.forward_ap['j_2_r_l'] = Joint(r2_low, r2_low_back, 1)
		self.forward_ph['j_2_f_r'] = Joint(f2_low_front, f2_low, -1)
		self.forward_ap['j_2_f_r'] = JointAmplit(-f2_low, -f2_upp, 1, 1, -f2_low_front)
		self.forward_ph['j_2_m_l'] = JointAmplit(m2_low, m2_low, 1, 1)
		self.forward_ap['j_2_m_l'] = JointAmplit(m2_low, m2_upp, 1, 1)
		self.forward_ph['j_2_r_r'] = Joint(-r2_low, -r2_low_back, 1)
		self.forward_ap['j_2_r_r'] = JointAmplit(-r2_low_back, -r2_upp, 1, 1, r2_low)


		self.forward_ph['j_3_f_l'] = Joint(f3_low, f3_upp, 1)
		self.forward_ap['j_3_f_l'] = Joint(-f3_upp, f3_low, -1)
		self.forward_ph['j_3_f_r'] = Joint(-f3_upp, f3_low, -1)
		self.forward_ap['j_3_f_r'] = Joint(f3_low, f3_upp, 1)

		self.forward_ph['j_3_m_l'] = Joint(m3_upp, m3_upp, 1)
		self.forward_ap['j_3_m_l'] = Joint(m3_upp, m3_upp, 1)
		self.forward_ph['j_3_m_r'] = Joint(m3_upp, m3_upp, 1)
		self.forward_ap['j_3_m_r'] = Joint(m3_upp, m3_upp, 1)

		self.forward_ph['j_3_r_l'] = Joint(-r3_upp, r3_low, -1)
		self.forward_ap['j_3_r_l'] = Joint(r3_low, r3_upp, 1)
		self.forward_ph['j_3_r_r'] = Joint(r3_low, r3_upp, 1)
		self.forward_ap['j_3_r_r'] = Joint(-r3_upp, r3_low, -1)

	def generate_first_step(self):
		self.first_stp = {}  # first step functions

		self.first_stp['j_1_f_l'] = Joint(0, -f1_low, -1)
		self.first_stp['j_1_f_r'] = Joint(0, f1_upp, -1)
		self.first_stp['j_1_m_l'] = Joint(0, m1_upp, 1)
		self.first_stp['j_1_m_r'] = Joint(0, -m1_low, 1)
		self.first_stp['j_1_r_l'] = Joint(0, -r1_low, -1)
		self.first_stp['j_1_r_r'] = Joint(0, r1_upp, -1)
											
		self.first_stp['j_2_f_l'] = JointAmplit(0, f2_upp, 1, 1, f2_low_front)
		self.first_stp['j_2_m_r'] = JointAmplit(m2_low, m2_upp, -1, 1)
		self.first_stp['j_2_r_l'] = JointAmplit(0, r2_upp, 1, 1, r2_low)
		self.first_stp['j_2_f_r'] = Joint(0, f2_low, -1)
		self.first_stp['j_2_m_l'] = Joint(0, m2_low, 1)
		self.first_stp['j_2_r_r'] = Joint(0, -r2_low_back, 1)

		self.first_stp['j_3_f_l'] = Joint(f3_low, f3_upp, 1)
		self.first_stp['j_3_f_r'] = Joint(0, f3_low, 1)
		self.first_stp['j_3_m_l'] = Joint(0, m3_upp, 1)
		self.first_stp['j_3_m_r'] = Joint(0, m3_upp, 1)
		self.first_stp['j_3_r_l'] = Joint(0, 0, -1)
		self.first_stp['j_3_r_r'] = Joint(r3_low, r3_upp, 1)

	def generate_left_turn(self):
		self.left_ph = {}  # left turn phase functions
		self.left_ap = {}  # left turn anti phase functions

		self.left_ph['j_1_f_l'] = Joint(0, 0, -1)
		self.left_ap['j_1_f_l'] = Joint(0, 0, 1)
		self.left_ph['j_1_f_r'] = Joint(f1_low, f1_upp, -1)
		self.left_ap['j_1_f_r'] = Joint(-f1_upp, -f1_low, 1)
		self.left_ph['j_1_m_l'] = Joint(0, 0, 1)
		self.left_ap['j_1_m_l'] = Joint(0, 0, -1)
		self.left_ph['j_1_m_r'] = Joint(0, 0, 1)
		self.left_ap['j_1_m_r'] = Joint(0, 0, -1)
		self.left_ph['j_1_r_l'] = Joint(0, 0, -1)
		self.left_ap['j_1_r_l'] = Joint(0, 0, 1)
		self.left_ph['j_1_r_r'] = Joint(r1_low, r1_upp, -1)
		self.left_ap['j_1_r_r'] = Joint(-r1_upp, -r1_low, 1)

	def generate_right_turn(self):
		self.right_ph = {}  # right turn phase functions
		self.right_ap = {}  # right turn anti phase functions

		self.right_ph['j_1_f_l'] = Joint(-f1_upp, -f1_low, -1)
		self.right_ap['j_1_f_l'] = Joint(f1_low, f1_upp, 1)
		self.right_ph['j_1_f_r'] = Joint(0, 0, -1)
		self.right_ap['j_1_f_r'] = Joint(0, 0, 1)
		self.right_ph['j_1_m_l'] = Joint(0, 0, 1)
		self.right_ap['j_1_m_l'] = Joint(0, 0, -1)
		self.right_ph['j_1_m_r'] = Joint(0, 0, 1)
		self.right_ap['j_1_m_r'] = Joint(0, 0, -1)
		self.right_ph['j_1_r_l'] = Joint(-r1_upp, -r1_low, -1)
		self.right_ap['j_1_r_l'] = Joint(r1_low, r1_upp, 1)
		self.right_ph['j_1_r_r'] = Joint(0, 0, -1)
		self.right_ap['j_1_r_r'] = Joint(0, 0, 1)

class Joint:
	def __init__(self, low, upp, 
		invert):

		self.low = low
		self.upp = upp
		self.invert = invert

	def get(self, x):
		return self.invert * interp(x, [0, 1], [self.low, self.upp])

class JointAmplit:
	def __init__(self, low, upp, 
		invert, phase, change_low = 0):

		self.low = low
		self.upp = upp
		self.invert = invert
		self.phase = phase
		self.change_low = change_low

	def get(self, x):
		if x < 0.5:
			return self.phase * self.invert * interp(x, [0, 0.5], [self.low, self.upp])
		else:
			if (self.change_low == 0):
				return -1 * self.phase * self.invert * interp(x, [0.5, 1], [-self.upp, -self.low])
			else:
				return -1 * self.phase * self.invert * interp(x, [0.5, 1], [-self.upp, -self.change_low])


if __name__ == '__main__':
	rospy.init_node('walk_node')
	robot = Ant()
	walk_control = WalkControl(robot)

	rospy.loginfo('\n Key controls:  \n \
		UP - start moving forward, \n \
		DOWN - stop moving forward, \n \
		LEFT - turn left, stop turning right, \n \
		RIGHT - turn right, stop turning left, \n \
		R - reset model position, \n \
		Z - set speed to slow, \n \
		X - set speed to medium, \n \
		C - set speed to fast, \n \
		Ctrl + C - exit simulation.')

	walk_control.start()

	while not rospy.is_shutdown():
		rospy.sleep(1)