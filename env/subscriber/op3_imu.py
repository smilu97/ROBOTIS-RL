from . import RosSubscriber

from sensor_msgs.msg import JointState

from sensor_msgs.msg import Imu

class Op3ImuSubscriber(RosSubscriber):
    def __init__(self):
        super().__init__('/robotis_op3/imu', Imu, 10)
