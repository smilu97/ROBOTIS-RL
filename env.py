import numpy as np
import os
import subprocess
import sys
import rospy
import op3
import time

from ros import RosController
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty

class Environment:
    def __init__(self, launchfile):
        self.ros = RosController(launchfile)
        self.target_pos = np.zeros(len(op3.controller_names))
    
    def calculate_observation(self, data):
        if data is None:
            return np.zeros(3 * len(op3.links)), False
        # Order positions by name
        pose_dict = {}
        for index, name in enumerate(data.name):
            pose_dict[name] = data.pose[index].position

        # Collect position numbers in 1-D array
        state = np.zeros(3 * len(op3.links))
        for index, name in enumerate(op3.links):
            p = pose_dict.get(name)
            if p is not None:
                state[index * 3 + 0] = p.x
                state[index * 3 + 1] = p.y
                state[index * 3 + 2] = p.z
        
        # Determine if robot fell down
        done = state[2] < 0.16
        
        return state, done
        
    def apply_action(self, action):
        diff = 0.001
        next_pos = np.array(self.target_pos)
        for i in range(len(op3.controller_names)):
            if action[i] == 1:
                next_pos[i] += diff
            elif action[i] == 2:
                next_pos[i] -= diff
        next_pos = np.maximum(-0.5, next_pos)
        next_pos = np.minimum( 0.5, next_pos)
        self.ros.publish_action(next_pos)
        self.target_pos = next_pos

    def step(self, action):
        # self.ros.unpause()

        for _ in range(10):
            self.apply_action(action)

        data = self.ros.get_link_states()
        # self.ros.pause()
        state, done = self.calculate_observation(data)
        reward = -1000 if done else state[2]
        return np.asarray(state), reward, done, {}

    def reset(self):
        self.target_pos = np.zeros(len(op3.controller_names))
        self.ros.wait_for_controllers()
        self.ros.reset()
        self.ros.unpause()
        data = self.ros.get_link_states()
        # self.ros.pause()
        state, done = self.calculate_observation(data)
        return np.asarray(state)
