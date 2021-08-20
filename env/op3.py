import numpy as np
import os
import subprocess
import sys
import rospy
import time

from .constants import op3_module_names, controller_names
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from controller_manager_msgs.srv import ListControllers
from robotis_controller_msgs.srv import SetJointModule, SetJointModuleRequest
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState, Imu

from .ros import RosController
from .proxy.list_controllers import ListControllersProxy

from .subscriber.link_states import LinkStatesSubscriber
from .subscriber.op3_imu import Op3ImuSubscriber
from .subscriber.op3_joint_states import Op3JointStatesSubscriber

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
        launchfile = None,
        subscribe_link_states = True,
        subscribe_imu = True,
        subscribe_joint_states = True,
        random_port=True,
    ):
        super(Op3Controller, self).__init__(launchfile, randomize_port=random_port)
        self.prev_action = np.zeros(20)

        self.list_controllers = ListControllersProxy()
        self.wait_controllers()
        self.publisher = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=20)

        self.link_states = LinkStatesSubscriber()
        self.joint_states = Op3JointStatesSubscriber()
        self.imu = Op3ImuSubscriber()
        self.subscribers = [self.link_states, self.joint_states, self.imu]

    def reset(self):
        self.reset_sim()
        super().reset()

        # Reset controller goal states into zeros
        self.act(np.zeros(len(op3_module_names)))
        
        # Reset all subscriber states
        for sub in self.subscribers: sub.reset()

        self.wait_states()
    
    def act(self, action):
        state = JointState()
        state.name = op3_module_names
        state.position = action[:len(op3_module_names)]

        self.publisher.publish(state)
        
    def wait_controllers(self):
        '''
        JointController들중 아직 활성화되지 않은 것이 있을 수 있기 때문에, 모든 컨트롤러가 응답할 때 까지 대기
        '''
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
    
    def wait_states(self):
        '''
        환경을 리셋하고 난 뒤 맨 처음 로봇의 상태를 받아오기 위해 기다림
        '''
        while True:
            if self.link_states.latest is not None \
                and self.imu.latest is not None \
                and self.joint_states.latest is not None:
                break
            self.iterate(40)
