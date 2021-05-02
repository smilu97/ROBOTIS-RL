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
        self.target_pos = np.zeros(len(op3.op3_module_names))
        self.prev_x = 0.0
        self.observation_size = 3*len(op3c.op3_module_names)+10
        self.action_size = len(op3c.op3_module_names)
        self.pause_sim = True

        sl = len(op3c.op3_module_names)

        self.action_range = np.array([
            45, # head_tilt
            45, # l_ank_pitch
            45, # l_ank_roll
            45, # l_el
            70, # l_hip_pitch
            45, # l_hip_roll
            20, # l_hip_yaw
            90, # l_knee
            45, # l_sho_pitch
            45, # l_sho_roll
            45, # r_ank_pitch
            45, # r_ank_roll
            45, # r_el
            70, # r_hip_pitch
            45, # r_hip_roll
            20, # r_hip_yaw
            90, # r_knee
            45, # r_sho_pitch
            45, # r_sho_roll
        ]) / 180 * np.pi

        self.action_space = gym.spaces.Box(low=-self.action_range, high=self.action_range, shape=(sl,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3*sl+10,), dtype=np.float32)
    
    def get_observation(self):
        joint_states = self.op3.get_joint_states()
        joint_dict = {}
        if joint_states is not None:
            for index, name in enumerate(joint_states.name):
                joint_dict[name] = [
                    joint_states.position[index],
                    joint_states.velocity[index] * 0.01,
                    joint_states.effort[index] * 0.05,
                ]
        state = np.empty(3 * len(op3c.op3_module_names) + 10) # len: 70
        pad = np.zeros(3)
        sl = len(op3c.op3_module_names)
        for index, name in enumerate(op3c.op3_module_names):
            jd = joint_dict.get(name, pad)
            state[index] = jd[0]
            state[sl + index] = jd[1]
            state[2*sl + index] = jd[2]
        self.target_pos = state[:3*len(op3c.op3_module_names):3]
        state[(3*len(op3c.op3_module_names)):] = serialize_imu(self.op3.get_imu())
        
        return state
    
    def get_current_x(self):
        return self.op3.get_imu().orientation.x
    
    def get_reward(self, done):
        x = self.get_current_x()
        reward = x - self.prev_x + 0.01
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

    def step(self, action):
        if self.pause_sim: self.op3.unpause()
        self.apply_action(action)
        time.sleep(1.0 / 60.0)
        if self.pause_sim: self.op3.pause()
        done = self.get_done()
        return (
            self.get_observation(),
            self.get_reward(done),
            done,
            {}
        )

    def reset(self):
        self.target_pos *= 0
        self.apply_action(self.target_pos)
        # time.sleep(0.5)
        self.op3.pause()
        self.op3.reset_sim()
        time.sleep(0.1)
        self.op3.unpause()
        while True:
            if self.op3.updated_imu:
                break
            time.sleep(0.01)
        if self.pause_sim: self.op3.pause()
        return self.get_observation()
