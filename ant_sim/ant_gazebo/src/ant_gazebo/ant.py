import rospy
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
 
class Ant:
    def __init__(self, ns='/ant/'):
        self.ns = ns
        self.joints = None
        self.angles = None

        self._sub_joints = rospy.Subscriber(
             ns + 'joint_states', JointState, self.get_joint_states, queue_size=1)

        while not rospy.is_shutdown():
            if self.joints is not None:
                break
            rospy.sleep(0.1)

        self._pub_joints = {}
        self.centered_angles = {}
        for j in self.joints:
            p = rospy.Publisher(
                ns + j + '_position_controller/command', Float64, queue_size=1)
            self._pub_joints[j] = p
            self.centered_angles[j] = 0.0

        rospy.sleep(1)
        

    def get_joint_states(self, joint_states):
        if self.joints is None:
            #rospy.loginfo(joint_states.name)
            self.joints = joint_states.name
        #rospy.loginfo(joint_states.position)
        self.angles = joint_states.position

    def get_angles(self):
        if self.joints is None:
            return None
        if self.angles is None:
            return None
        return dict(zip(self.joints, self.angles))

    def set_angles(self, angles):
        for j, v in angles.items():  
            self._pub_joints[j].publish(v)

    def center_angles(self):
        #rospy.loginfo("Centering angles.")
        start_angles = self.get_angles()
        start_time = time.time()
        stop_time = start_time + 1
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            t = time.time()
            if t > stop_time:
                break
            ratio = (t - start_time) / 1
            interp = {}
            joints = self.centered_angles.keys()
            for j in joints:
                interp[j] = self.centered_angles[j] * ratio + \
                start_angles[j] * (1 - ratio)
            self.set_angles(interp)
            r.sleep()



