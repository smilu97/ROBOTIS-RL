import rospy

from . import RosProxy

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Twist, Pose

class SpawnOp3Proxy(RosProxy):
    def __init__(self):
        super().__init__('/gazebo/spawn_urdf_model', SpawnModel)

    def __call__(self):
        model_xml = rospy.get_param('robot_description')
        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 0.285
        self.spawn_model_proxy('robotis_op3', model_xml, '', initial_pose, 'world')
