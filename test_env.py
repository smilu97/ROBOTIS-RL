#!/usr/bin/env python

import numpy as np
import time
import gym
import env
import pybulletgym
from matplotlib import pyplot as plt

def main():
    env = gym.make('RobotisOp3-v0')
    epochs = 1000
    episode = 1
    sl = env.action_space.shape[0]
    
    if False:
        env.render(mode='human')

    for num_epoch in range(epochs):
        print('epsiode {} start'.format(episode))
        env.reset()
        done = False
        acc_reward = 0.0
        num_steps = 0
        while not done:
            action = 2 * np.random.rand(sl) - 1
            state, reward, done, info = env.step(action)
            acc_reward += reward
            num_steps += 1
        
        print('reward:', acc_reward)
        print('num steps:', num_steps)
        episode += 1

if __name__ == '__main__':
    main()
