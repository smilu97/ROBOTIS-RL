from . import RosSubscriber

from sensor_msgs.msg import JointState

class Op3JointStatesSubscriber(RosSubscriber):
    def __init__(self):
        super().__init__('/robotis/present_joint_states', JointState, 10)
