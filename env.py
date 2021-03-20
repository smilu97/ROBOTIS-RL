import numpy as np
import os
import subprocess
import sys
import rospy
import op3
import time
import op3constant as op3c

from op3 import Op3Controller
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState, Imu

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

class Environment:
    def __init__(self, launchfile):
        self.op3 = Op3Controller(launchfile)
        self.target_pos = np.zeros(len(op3.controller_names))
        self.prev_x = 0.0
        self.observation_size = 3*len(op3c.op3_module_names)+10
        self.action_size = op3c.op3_module_names
    
    def get_observation(self):
        joint_states = self.op3.get_joint_states()
        joint_dict = {}
        if joint_states is not None:
            for index, name in enumerate(joint_states.name):
                joint_dict[name] = [
                    joint_states.position[index],
                    joint_states.velocity[index],
                    joint_states.effort[index]
                ]
        state = np.empty(3 * len(op3c.op3_module_names) + 10) # len: 70
        pad = np.zeros(3)
        for index, name in enumerate(op3c.op3_module_names):
            state[(3*index):(3*index+3)] = joint_dict.get(name, pad)
        state[(3*len(op3c.op3_module_names)):] = serialize_imu(self.op3.get_imu())
        
        return state
    
    def get_reward(self):
        x = self.op3.get_imu().orientation.x
        reward = x - self.prev_x
        self.prev_x = x
        return reward
    
    def get_done(self):
        target = 'robotis_op3::body_link'
        thres = 0.1

        link_states = self.op3.get_link_states()
        for index, name in enumerate(link_states.name):
            if name == target:
                val = link_states.pose[index].position.z
                return val < thres
        return False
        
    def apply_action(self, action):
        self.op3.publish_action2(action)
        return

        diff = 0.001
        next_pos = np.array(self.target_pos)
        for i in range(len(op3.controller_names)):
            if action[i] == 1:
                next_pos[i] += diff
            elif action[i] == 2:
                next_pos[i] -= diff
        next_pos = np.maximum(-0.5, next_pos)
        next_pos = np.minimum( 0.5, next_pos)
        self.op3.publish_action(next_pos)
        self.target_pos = next_pos

    def step(self, action):
        # self.op3.unpause()
        self.apply_action(action)
        # self.op3.pause()
        return (
            self.get_observation(),
            self.get_reward(),
            self.get_done(),
            {}
        )

    def reset(self):
        self.target_pos = np.zeros(len(op3.controller_names))
        self.op3.reset()
        self.op3.unpause()
        # self.op3.pause()
        return self.get_observation()
