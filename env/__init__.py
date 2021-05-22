import numpy as np
import os
import subprocess
import sys
import rospy
import time
import gym

from . import op3constant as op3c
from .op3 import Op3Controller, serialize_imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState, Imu

class OP3Env(gym.Env):
    def __init__(self, step_size=0.02):
        self.op3 = Op3Controller()
        self.target_pos = np.zeros(len(op3c.op3_module_names))
        self.prev_x = 0.0
        self.observation_size = 3*len(op3c.op3_module_names)+10
        self.action_size = len(op3c.op3_module_names)
        self.pause_sim = False
        self.reward_debug = np.zeros(4)

        sl = len(op3c.op3_module_names)
        self.sl = sl
        self.op3c = op3c
        self.action_range = np.array(op3c.joint_ranges) / 180 * np.pi
        self.action_space = gym.spaces.Box(low=-self.action_range, high=self.action_range, shape=(sl,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3*sl+10,), dtype=np.float32)
        self.step_size = step_size
    
    def render(self, **kwargs):
        self.op3.render(**kwargs)
    
    def get_observation(self):
        joint_states = self.op3.latest_joint_states
        joint_dict = {}
        if joint_states is not None:
            for index, name in enumerate(joint_states.name):
                joint_dict[name] = (
                    joint_states.position[index],
                    joint_states.velocity[index] * 0.01,
                    joint_states.effort[index] * 0.05,
                )
        state = np.empty(3 * len(op3c.op3_module_names) + 10) # len: 70
        pad = np.zeros(3)
        for index, name in enumerate(op3c.op3_module_names):
            jd = joint_dict.get(name, pad)
            state[index] = jd[0]
            state[self.sl + index] = jd[1]
            state[2*self.sl + index] = jd[2]
        self.target_pos = state[:3*len(op3c.op3_module_names):3]
        state[(3*len(op3c.op3_module_names)):] = serialize_imu(self.op3.latest_imu)
        
        return state
    
    def count_stuck(self, obs):
        position = obs[:self.sl]
        return np.sum(position <= -self.action_range) + np.sum(position >= self.action_range)
    
    alive_bonus = 3.0
    energy_cost = -5.0
    stuck_cost = -1.0
    progress_bonus = 100.0

    def get_reward(self, obs, position, done):
        dx = position.x - self.prev_x
        self.prev_x = position.x
        progress = dx * self.progress_bonus
        alive = position.z * self.alive_bonus
        energy = np.sum(np.square(obs[self.sl : 2*self.sl])) * self.energy_cost
        stuck = self.count_stuck(obs) * self.stuck_cost
        rewards = np.array((progress, alive, energy, stuck))

        return np.sum(rewards)
    
    def get_done(self, position):
        return position.z < 0.2
    
    def get_position(self):
        target = 'robotis_op3::body_link'
        link_states = self.op3.latest_link_states
        for index, name in enumerate(link_states.name):
            if name == target:
                return link_states.pose[index].position
        return 0.0

    def step(self, action):
        if self.pause_sim: self.op3.unpause()
        self.op3.act(action * self.action_range)
        time.sleep(self.step_size)
        if self.pause_sim: self.op3.pause()
        position = self.get_position()
        done = self.get_done(position)
        obs = self.get_observation()
        if np.any(np.isnan(obs)):
            raise Exception('robot broken down!')
        reward = self.get_reward(obs, position, done)

        return (
            obs,
            reward,
            done,
            {}
        )

    def reset(self):
        self.target_pos *= 0
        self.prev_x = 0.0
        self.op3.act(self.target_pos)
        self.op3.pause()
        self.op3.reset_sim()
        self.op3.unpause()
        self.op3.wait_imu()
        if self.pause_sim: self.op3.pause()
        return self.get_observation()
    
    def reset_op3(self):
        self.op3 = Op3Controller()

gym.envs.register(
    id='RobotisOp3-v0',
    entry_point=OP3Env
)
