#!/usr/bin/env python

import numpy as np
import time
import gym
import env
import pybulletgym
from matplotlib import pyplot as plt

def main():
    env = gym.make('RobotisOp3-v0', random_port=False, print_rewards=True)
    epochs = 1000
    episode = 1
    sl = env.action_space.shape[0]
    
    if True:
        env.render(mode='human')

    for num_epoch in range(epochs):
        print('epsiode {} start'.format(episode))
        obs = env.reset()
        done = False
        acc_reward = 0.0
        num_steps = 0
        while not done:
            action = 2 * np.random.rand(sl) - 1
            obs, reward, done, info = env.step(action)
            acc_reward += reward
            num_steps += 1
        
        print('reward:', acc_reward)
        print('num steps:', num_steps)
        episode += 1

if __name__ == '__main__':
    main()
