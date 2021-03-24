import numpy as np
import os
import subprocess
import sys
import rospy
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from controller_manager_msgs.srv import ListControllers
from robotis_controller_msgs.srv import SetJointModule, SetJointModuleRequest
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState, Imu

from op3constant import *

from ros import RosController

def serialize_imu(data):
    '''
    Serialize sensor_msgs/Imu into np.array(float32[10])
    '''
    if data is None:
        return np.zeros(10)

    ot  = data.orientation # geometry_msgs/Quarternion
    av  = data.angular_velocity # geometry_msgs/Vector3
    la  = data.linear_acceleration # geometry_msgs/Vector3

    return np.array([
        ot.x,ot.y,ot.z,ot.w,
        av.x,av.y,av.z,
        la.x,la.y,la.z,
    ], dtype=np.float32) # len: 10

class Op3Controller(RosController):
    def __init__(
        self,
        launchfile,
        subscribe_link_states = True,
        subscribe_imu = True,
        subscribe_joint_states = True,
    ):
        super(Op3Controller, self).__init__(launchfile)

        self.latest_link_states = None
        self.latest_imu = None
        self.latest_joint_states = None
        self.prev_action = np.zeros(20)
        self.updated_imu = False

        def link_states_cb(data):
            self.latest_link_states = data
        def imu_cb(data):
            self.updated_imu = True
            self.latest_imu = data
        def joint_states_cb(data):
            self.latest_joint_states = data

        self.publishers = [rospy.Publisher(topic, Float64, queue_size=20) for topic in command_topics]
        self.link_states_publisher = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=20)
        if subscribe_link_states:
            self.link_states_subscriber = rospy.Subscriber('/gazebo/link_states', LinkStates, link_states_cb, queue_size=10)
        if subscribe_joint_states:
            self.joint_states_subscriber = rospy.Subscriber('/robotis/present_joint_states', JointState, joint_states_cb, queue_size=10)
        if subscribe_imu:
            self.imu_subscriber = rospy.Subscriber('/robotis_op3/imu', Imu, imu_cb, queue_size=10)
        self.list_controllers = rospy.ServiceProxy('/robotis_op3/controller_manager/list_controllers', ListControllers)
        self.reset_direct_motion_srv = rospy.ServiceProxy('/robotis/gym/reset_motion', Empty)

        self.wait_for_controllers()

    def reset(self):
        self.reset_goal()
        self.updated_imu = False
        super(Op3Controller, self).reset()

    def get_link_states(self):
        return self.latest_link_states
    
    def get_imu(self):
        return self.latest_imu
    
    def get_joint_states(self):
        return self.latest_joint_states
    
    def publish_action(self, action):
        for index, pub in enumerate(self.publishers):
            value = Float64()
            value.data = action[index]
            pub.publish(value)
    
    def publish_action2(self, action):
        state = JointState()
        state.name = op3_module_names
        state.position = action[:len(op3_module_names)]
        self.link_states_publisher.publish(state)
    
    def reset_goal(self):
        state = JointState()
        state.name = op3_module_names
        state.position = np.zeros(len(op3_module_names))
        self.link_states_publisher.publish(state)
    
    def wait_for_controllers(self):
        rospy.wait_for_service('/robotis_op3/controller_manager/list_controllers')
        ctrls = None
        while True:
            ctrls = self.list_controllers().controller
            ctrl_names = [x.name for x in ctrls]
            flag = True
            for name in controller_names:
                if name not in ctrl_names:
                    flag = False
                    break
            if flag: break
