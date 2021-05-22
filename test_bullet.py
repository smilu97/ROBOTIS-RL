#!/usr/bin/env python

import numpy as np
import gym
import pybulletgym

def policy(state):
    return np.zeros(17)

def main():
    env = gym.make('HumanoidPyBulletEnv-v0')
    env.render()
    while True:
        state = env.reset()
        while True:
            action = policy(state)
            state, reward, done, info = env.step(action)
            if done: break

if __name__ == '__main__':
    main()
