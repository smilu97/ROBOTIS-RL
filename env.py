import numpy as np
import os
import subprocess
import sys
import rospy
import op3
import time
import op3constant as op3c
import gym

from op3 import Op3Controller, serialize_imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState, Imu

class Op3Environment(gym.Env):
    def __init__(self, launchfile):
        self.op3 = Op3Controller(launchfile)
        self.target_pos = np.zeros(len(op3.controller_names))
        self.prev_x = 0.0
        self.observation_size = 3*len(op3c.op3_module_names)+10
        self.action_size = len(op3c.op3_module_names)

        sl = len(op3c.op3_module_names)
        self.action_space = gym.spaces.Box(low=0.5, high=0.5, shape=(sl,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3*sl+10,), dtype=np.float32)
    
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
    
    def get_current_x(self):
        target = 'robotis_op3::body_link'
        states = self.op3.get_link_states()
        for index in range(len(states.name)):
            if states.name[index] == target:
                return states.pose[index].position.x
        return 0.0
        return self.op3.get_imu().orientation.x
    
    def get_reward(self):
        x = self.get_current_x()
        reward = x - self.prev_x
        self.prev_x = x
        return reward
    
    def get_done(self):
        target = 'robotis_op3::body_link'
        thres = 0.2

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
        self.op3.unpause()
        self.apply_action(action)
        self.op3.pause()
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
        while True:
            if self.op3.updated_imu:
                break
            time.sleep(0.01)
        return self.get_observation()
