#!/usr/bin/env python

import numpy as np
import gym
import pybulletgym
import time

def main():
    env = gym.make('HalfCheetahPyBulletEnv-v0')
    env.render()
    sl = env.action_space.shape[0]
    random_action = False
    while True:
        state = env.reset()
        while True:
            if random_action:
                action = 2 * np.random.rand(sl) - 1
            else:
                action = np.zeros(sl)
            state, reward, done, info = env.step(action)
            time.sleep(1/60)
            if done: break

if __name__ == '__main__':
    main()
