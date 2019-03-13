import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
 

class Ant:

    def __init__(self, ns='/ant/'):
        self.ns = ns
        self.joints = None
        self.angles = None

        self._sub_joints = rospy.Subscriber(
            ns + 'joint_states', JointState, self._cb_joints, queue_size=1)
        while not rospy.is_shutdown():
            if self.joints is not None:
                break
            rospy.sleep(0.1)

        self._pub_joints = {}
        for j in self.joints:
            p = rospy.Publisher(
                ns + j + '_position_controller/command', Float64, queue_size=1)
            self._pub_joints[j] = p

        rospy.sleep(1)
        

    def _cb_joints(self, msg):
        if self.joints is None:
            rospy.loginfo(msg.name)
            self.joints = msg.name
        self.angles = msg.position

    def get_angles(self):
        if self.joints is None:
            return None
        if self.angles is None:
            return None
        return dict(zip(self.joints, self.angles))

    def set_angles(self, angles):
        for j, v in angles.items():  
            self._pub_joints[j].publish(v)

