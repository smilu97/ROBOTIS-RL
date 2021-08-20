import numpy as np
import os
import subprocess
import sys
import rospy
import time
import gym

from . import constants as op3c
from .op3 import Op3Controller, serialize_imu
from .params import params

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState, Imu

progress_bonus = params.progress_bonus
accel_bonus = params.accel_bonus
alive_bonus = params.alive_bonus
outroute_cost = params.outroute_cost
velocity_cost = params.velocity_cost
height_bonus = params.height_bonus
effort_cost = params.effort_cost
stuck_cost = params.stuck_cost
action_modify_rate = params.action_modify_rate
reference_cycle = params.reference_cycle
reference_weight = params.reference_weight

def unsym_sin(x, w=0.35):
    r = np.sin(x)
    if r < 0:
        r *= w
    return r

class OP3Env(gym.Env):
    def __init__(
        self,
        human_bias=False,
        step_size=0.02,
        random_port=True,
        print_rewards=False,
        use_bias=True
    ):
    
        self.print_rewards = print_rewards
        self.human_bias = human_bias
        self.step_size = step_size
        self.num_mod = len(op3c.op3_module_names)
        self.T = 1.0

        self.action_bias = np.array(op3c.joint_bias) / 180 * np.pi
        self.action_range = np.array(op3c.joint_ranges) / 180 * np.pi

        self.acc_action = np.zeros(self.num_mod, dtype=np.float32)
        self.reward_debug = np.zeros(3)
        
        self.op3 = Op3Controller(random_port=random_port)
        self.reset_variables()

        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(12,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(24,), dtype=np.float32)

        if not use_bias:
            self.action_bias[:] = 0
    
    def reset_variables(self):
        self.prev_x = 0.0
        self.prev_dx = 0.0
        self.prev_z = None
        self.t = 0.0
        self.op3.last_clock = 0
        self.op3.latest_joint_states = None

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
        self.positions  = np.array([joint_dict[name][0] for name in op3c.op3_obs_module_names], dtype=np.float32)
        # velocities = np.array([joint_dict[name][1] for name in op3c.op3_obs_module_names])
        self.efforts    = [joint_dict[name][2] for name in op3c.op3_obs_module_names]

        imu = serialize_imu(self.op3.latest_imu)
        
        t = 2 * np.pi * self.t / self.T
        return np.concatenate([self.positions, imu, [np.sin(t), np.cos(t)]])
    
    def count_stuck(self, obs):
        position = obs[:self.num_mod]
        p1 = self.action_bias - self.action_range
        p2 = self.action_bias + self.action_range
        return np.sum(position <= p1) + np.sum(position >= p2)

    def get_reward(self, obs, position, done):
        dx = position.x - self.prev_x
        ddx = dx - self.prev_dx
        # dz = 0 if self.prev_z is None else position.z - self.prev_z
        self.prev_x = position.x
        # self.prev_z = position.z
        self.prev_dx = dx

        # accel = ddx * accel_bonus
        progress = dx * progress_bonus
        # height = position.z *.height_bonus
        alive = -5.0 if done else alive_bonus
        # outroute = abs(position.y) * outroute_cost
        # velocity = np.sum(np.abs(obs[self.num_mod : 2*self.num_mod])) * velocity_cost
        effort = np.sum(np.abs(self.efforts)) * effort_cost
        # stuck = self.count_stuck(obs) * self.stuck_cost

        rewards = np.array((progress, alive, effort))
        self.reward_debug += rewards

        return np.sum(rewards)
    
    def get_done(self, position):
        return self.t >= 50.0 or position.z < 0.2
    
    def get_position(self):
        target = 'robotis_op3::body_link'
        link_states = self.op3.latest_link_states
        for index, name in enumerate(link_states.name):
            if name == target:
                return link_states.pose[index].position
        return 0.0
    
    def get_human_bias_reference(self):
        if not self.human_bias:
            return 0.0
            
        t = 2 * np.pi * self.t / self.T
        ank_pitch = 30
        hip_pitch = 60
        hip_roll  = 10
        knee      = 45
        sho_pitch = 45
        return np.array([
            0, # head_tilt 0
            -ank_pitch * unsym_sin(t, -0.35), # l_ank_pitch 1
            0, # l_ank_roll 2
            0, # l_el 3
            -hip_pitch * unsym_sin(t), # l_hip_pitch 4
            hip_roll * unsym_sin(t, 1), # l_hip_roll 5
            0, # l_hip_yaw 6
            knee * unsym_sin(t, 0), # l_knee 7
            sho_pitch * unsym_sin(t, 1), # l_sho_pitch 8
            0, # l_sho_roll 9
            ank_pitch * unsym_sin(t+np.pi, -0.35), # r_ank_pitch 10
            0, # r_ank_roll 11
            0, # r_el 12
            hip_pitch * unsym_sin(t+np.pi), # r_hip_pitch 13
            hip_roll * unsym_sin(t, 1), # r_hip_roll 14
            0, # r_hip_yaw 15
            -knee * unsym_sin(t+np.pi, 0), # r_knee 16
            -sho_pitch * unsym_sin(t+np.pi, 1), # r_sho_pitch 17
            0, # r_sho_roll 18
        ], dtype=np.float32) / 180 * np.pi * 0.5

    def step(self, action):
        action_modify_rate = params.action_modify_rate

        action = np.array([
            0, # head_tilt 0
            action[0], # l_ank_pitch 1
            action[1], # l_ank_roll 2
            0, # l_el 3
            action[2], # l_hip_pitch 4
            action[3], # l_hip_roll 5
            action[4], # l_hip_yaw 6
            action[5], # l_knee 7
            0, # l_sho_pitch 8
            0, # l_sho_roll 9
            action[6], # r_ank_pitch 3
            action[7], # r_ank_roll 11
            0, # r_el 12
            action[8], # r_hip_pitch 13
            action[9], # r_hip_roll 14
            action[10], # r_hip_yaw 15
            action[11], # r_knee 16
            0, # r_sho_pitch 17
            0, # r_sho_roll 18
        ], dtype=np.float32)
        action = np.minimum(1.0, np.maximum(-1.0, action))

        self.acc_action = self.acc_action * (1.0 - action_modify_rate) + action * action_modify_rate
        action_msg = self.acc_action * self.action_range + self.action_bias + self.get_human_bias_reference()
        action_msg = np.minimum(np.pi / 2, action_msg)
        action_msg = np.maximum(-np.pi / 2, action_msg)
        self.op3.act(action_msg)
        self.op3.iterate(10) # 200 Hz

        position = self.get_position()
        done = self.get_done(position)
        obs = self.get_observation()
        if np.any(np.isnan(obs)):
            raise Exception('robot broken down!')
        reward = self.get_reward(obs, position, done)
        self.t += 0.005

        if self.print_rewards and done:
            print('rewards:', self.reward_debug)
            self.reward_debug[:] = 0

        return (
            obs,
            reward,
            done,
            {}
        )

    def reset(self):
        self.reset_variables()
        self.op3.reset_sim()
        self.op3.reset()
        self.op3.wait_states()
        return self.get_observation()
    
    def reset_op3(self):
        self.op3 = Op3Controller()

gym.envs.register(
    id='RobotisOp3-v0',
    entry_point=OP3Env
)
